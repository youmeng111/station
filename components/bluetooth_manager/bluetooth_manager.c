#include "bluetooth_manager.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

/* NimBLE APIs */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

static const char* TAG = "BT_BROADCAST_MGR";

/* 外部函数声明 */
void ble_store_config_init(void);

/* 广播管理器全局状态 */
static bt_mgr_state_t g_manager_state = BT_MGR_STATE_UNINITIALIZED;
static SemaphoreHandle_t g_manager_mutex = NULL;

/* 设备分组管理 */
static device_group_t g_device_groups[MAX_DEVICE_GROUPS];
static uint8_t g_active_group_count = 0;

/* 广播控制 */
static broadcast_stats_t g_broadcast_stats = {0};
static uint32_t g_broadcast_interval_ms = BLE_ADV_INTERVAL_MIN;
static uint8_t g_repeat_count = BLE_ADV_REPEAT_COUNT;
static esp_timer_handle_t g_broadcast_timer = NULL;
static esp_timer_handle_t g_repeat_timer = NULL;

/* 回调函数 */
static broadcast_sent_cb_t g_broadcast_sent_cb = NULL;
static device_feedback_cb_t g_device_feedback_cb = NULL;
static inventory_result_cb_t g_inventory_result_cb = NULL;
static health_check_result_cb_t g_health_check_result_cb = NULL;
static fault_report_cb_t g_fault_report_cb = NULL;

/* 当前广播状态 */
static broadcast_state_t g_broadcast_state = BROADCAST_STATE_IDLE;
static broadcast_packet_t g_current_packet = {0};
static uint8_t g_current_repeat_count = 0;

/* 内部函数声明 */
static void nimble_host_task(void *param);
static void on_reset(int reason);
static void on_sync(void);
static int gap_event_handler(struct ble_gap_event *event, void *arg);
static void broadcast_timer_callback(void* arg);
static void repeat_timer_callback(void* arg);
static bt_mgr_err_t start_advertising_packet(const broadcast_packet_t *packet);
static bt_mgr_err_t stop_advertising(void);
static device_group_t* find_group_by_id(uint8_t group_id);
static void init_default_groups(void);

/* =================== 协议处理函数实现 =================== */

bool validate_broadcast_packet(const broadcast_packet_t *packet)
{
    if (!packet) return false;
    
    // 检查协议头
    if (packet->header[0] != BROADCAST_PACKET_HEADER_1 || 
        packet->header[1] != BROADCAST_PACKET_HEADER_2) {
        return false;
    }
    
    // 检查协议尾
    if (packet->tail != BROADCAST_PACKET_TAIL) {
        return false;
    }
    
    // 检查数据类型
    if (packet->type != 0xFF) {
        return false;
    }
    
    return true;
}

uint16_t pack_broadcast_packet(const broadcast_packet_t *packet, uint8_t *buffer, uint16_t buffer_size)
{
    if (!packet || !buffer || buffer_size < 31) {
        return 0;
    }
    
    uint8_t *ptr = buffer;
    uint8_t *start = ptr;
    
    // BLE广播数据格式：长度 + 类型 + 数据
    // 厂商数据格式：[长度][0xFF][厂商ID低字节][厂商ID高字节][厂商数据...]
    
    uint8_t payload_len = 2 + 2 + 1 + 1 + 2 + 1 + 8 + 1; // company_id(2) + header(2) + cmd_type(1) + group_id(1) + sequence(2) + priority(1) + params(8) + tail(1)
    
    // 厂商数据AD结构
    *ptr++ = payload_len + 3; // 长度：厂商ID(2) + 厂商数据(payload_len) + 类型(1)
    *ptr++ = 0xFF;            // 厂商数据类型
    *ptr++ = (packet->company_id & 0xFF);       // 厂商ID低字节
    *ptr++ = (packet->company_id >> 8) & 0xFF;  // 厂商ID高字节
    
    // 厂商数据内容
    *ptr++ = packet->header[0];
    *ptr++ = packet->header[1]; 
    *ptr++ = packet->cmd_type;
    *ptr++ = packet->group_id;
    *ptr++ = packet->sequence & 0xFF;
    *ptr++ = (packet->sequence >> 8) & 0xFF;
    *ptr++ = packet->priority;
    
    // 参数数据（8字节）
    memcpy(ptr, &packet->params, 8);
    ptr += 8;
    
    *ptr++ = packet->tail;
    
    uint16_t total_len = ptr - start;
    
    // 检查长度是否超过BLE广播数据限制
    if (total_len > 31) {
        ESP_LOGE(TAG, "Broadcast packet too large: %d bytes", total_len);
        return 0;
    }
    
    ESP_LOGD(TAG, "Packed broadcast packet: %d bytes (no checksum)", total_len);
    return total_len;
}

bool unpack_broadcast_packet(const uint8_t *buffer, uint16_t length, broadcast_packet_t *packet)
{
    if (!buffer || !packet || length < sizeof(broadcast_packet_t)) {
        return false;
    }
    
    memcpy(packet, buffer, sizeof(broadcast_packet_t));
    return validate_broadcast_packet(packet);
}

/* =================== 基础管理器接口 =================== */

bt_mgr_err_t bluetooth_manager_init(void)
{
    if (g_manager_state != BT_MGR_STATE_UNINITIALIZED) {
        ESP_LOGW(TAG, "Bluetooth manager already initialized");
        return BT_MGR_OK;
    }
    
    ESP_LOGI(TAG, "Initializing Bluetooth Broadcast Manager");
    
    // 创建互斥锁
    g_manager_mutex = xSemaphoreCreateMutex();
    if (!g_manager_mutex) {
        ESP_LOGE(TAG, "Failed to create manager mutex");
        return BT_MGR_ERR_INIT_FAILED;
    }
    
    // 初始化设备分组
    memset(g_device_groups, 0, sizeof(g_device_groups));
    g_active_group_count = 0;
    init_default_groups();
    
    // 初始化广播统计
    memset(&g_broadcast_stats, 0, sizeof(g_broadcast_stats));
    g_broadcast_stats.current_sequence = 1;
    
    // 初始化NimBLE
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize nimble port: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_manager_mutex);
        return BT_MGR_ERR_INIT_FAILED;
    }
    
    // 配置host回调函数
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    
    // 设置设备名称
    int rc = ble_svc_gap_device_name_set("ESP32_Broadcast_Station");
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set device name: %d", rc);
        vSemaphoreDelete(g_manager_mutex);
        return BT_MGR_ERR_INIT_FAILED;
    }
    
    g_manager_state = BT_MGR_STATE_INITIALIZED;
    ESP_LOGI(TAG, "Bluetooth Broadcast Manager initialized successfully");
    
    return BT_MGR_OK;
}

bt_mgr_err_t bluetooth_manager_start(void)
{
    if (g_manager_state != BT_MGR_STATE_INITIALIZED) {
        ESP_LOGE(TAG, "Bluetooth manager not initialized");
        return BT_MGR_ERR_NOT_INITIALIZED;
    }
    
    ESP_LOGI(TAG, "Starting Bluetooth Broadcast Manager");
    
    // 启动NimBLE host任务
    nimble_port_freertos_init(nimble_host_task);
    
    return BT_MGR_OK;
}

bt_mgr_err_t bluetooth_manager_stop(void)
{
    ESP_LOGI(TAG, "Stopping Bluetooth Broadcast Manager");
    
    // 停止广播
    stop_advertising();
    
    // 停止定时器
    if (g_broadcast_timer) {
        esp_timer_stop(g_broadcast_timer);
        esp_timer_delete(g_broadcast_timer);
        g_broadcast_timer = NULL;
    }
    
    if (g_repeat_timer) {
        esp_timer_stop(g_repeat_timer);
        esp_timer_delete(g_repeat_timer);
        g_repeat_timer = NULL;
    }
    
    // 停止NimBLE
    int rc = nimble_port_stop();
    if (rc == 0) {
        nimble_port_deinit();
    }
    
    g_manager_state = BT_MGR_STATE_INITIALIZED;
    return BT_MGR_OK;
}

bt_mgr_state_t bluetooth_manager_get_state(void)
{
    return g_manager_state;
}

/* =================== 广播控制接口 =================== */

bt_mgr_err_t bluetooth_manager_broadcast_command(const broadcast_command_t *cmd, broadcast_sent_cb_t sent_cb)
{
    if (!cmd) {
        return BT_MGR_ERR_INVALID_PARAM;
    }
    
    if (g_manager_state != BT_MGR_STATE_READY) {
        ESP_LOGE(TAG, "Manager not ready for broadcasting");
        return BT_MGR_ERR_NOT_INITIALIZED;
    }
    
    xSemaphoreTake(g_manager_mutex, portMAX_DELAY);
    
    // 构建广播数据包
    broadcast_packet_t packet = {0};
    packet.length = sizeof(broadcast_packet_t) - 1; // 不包括length字段本身
    packet.type = 0xFF; // 厂商数据类型
    packet.company_id = 0x02E5; // Espressif厂商ID
    packet.header[0] = BROADCAST_PACKET_HEADER_1;
    packet.header[1] = BROADCAST_PACKET_HEADER_2;
    packet.cmd_type = cmd->cmd_type;
    packet.group_id = cmd->group_id;
    packet.sequence = g_broadcast_stats.current_sequence++;
    packet.priority = cmd->priority;
    packet.tail = BROADCAST_PACKET_TAIL;
    
    // 复制命令参数
    memcpy(&packet.params, &cmd->params, sizeof(packet.params));
    
    // 保存当前数据包和回调
    memcpy(&g_current_packet, &packet, sizeof(packet));
    g_broadcast_sent_cb = sent_cb;
    g_current_repeat_count = 0;
    
    // 开始广播
    bt_mgr_err_t result = start_advertising_packet(&packet);
    
    if (result == BT_MGR_OK) {
        g_broadcast_stats.total_broadcasts++;
        g_broadcast_stats.last_broadcast_time = esp_timer_get_time() / 1000;
        ESP_LOGI(TAG, "Broadcasting command type=%d, group=%d, seq=%d", 
                 cmd->cmd_type, cmd->group_id, packet.sequence);
    } else {
        g_broadcast_stats.failed_broadcasts++;
        ESP_LOGE(TAG, "Failed to start broadcasting");
    }
    
    xSemaphoreGive(g_manager_mutex);
    
    return result;
}

bt_mgr_err_t bluetooth_manager_broadcast_all_set_color(uint8_t color, uint8_t blink_mode, uint32_t duration_ms)
{
    broadcast_command_t cmd = {0};
    cmd.cmd_type = BROADCAST_CMD_SET_COLOR;
    cmd.group_id = GROUP_ALL;
    cmd.priority = PRIORITY_NORMAL;
    cmd.params.led_params.color = color;
    cmd.params.led_params.blink_mode = blink_mode;
    cmd.params.led_params.duration_ms = duration_ms;
    
    return bluetooth_manager_broadcast_command(&cmd, NULL);
}

bt_mgr_err_t bluetooth_manager_broadcast_group_set_color(uint8_t group_id, uint8_t color, uint8_t blink_mode, uint32_t duration_ms)
{
    broadcast_command_t cmd = {0};
    cmd.cmd_type = BROADCAST_CMD_SET_COLOR;
    cmd.group_id = group_id;
    cmd.priority = PRIORITY_NORMAL;
    cmd.params.led_params.color = color;
    cmd.params.led_params.blink_mode = blink_mode;
    cmd.params.led_params.duration_ms = duration_ms;
    
    return bluetooth_manager_broadcast_command(&cmd, NULL);
}

bt_mgr_err_t bluetooth_manager_broadcast_battery_warning(uint8_t battery_level, uint16_t voltage_mv, uint8_t warning_type)
{
    broadcast_command_t cmd = {0};
    cmd.cmd_type = BROADCAST_CMD_BATTERY_WARNING;
    cmd.group_id = GROUP_ALL;
    cmd.priority = PRIORITY_HIGH;
    cmd.params.battery_warning.battery_level = battery_level;
    cmd.params.battery_warning.voltage_mv = voltage_mv;
    cmd.params.battery_warning.warning_type = warning_type;
    
    return bluetooth_manager_broadcast_command(&cmd, NULL);
}

bt_mgr_err_t bluetooth_manager_start_inventory_scan(uint32_t scan_id, uint8_t scan_type, uint8_t group_id, uint32_t timeout_ms)
{
    broadcast_command_t cmd = {0};
    cmd.cmd_type = BROADCAST_CMD_INVENTORY_SCAN;
    cmd.group_id = group_id;
    cmd.priority = PRIORITY_HIGH;
    cmd.params.inventory_params.scan_id = scan_id;
    cmd.params.inventory_params.scan_type = scan_type;
    cmd.params.inventory_params.timeout_ms = timeout_ms;
    
    ESP_LOGI(TAG, "Starting inventory scan ID=%lu, type=%d, group=%d, timeout=%lums", 
             scan_id, scan_type, group_id, timeout_ms);
    
    return bluetooth_manager_broadcast_command(&cmd, NULL);
}

bt_mgr_err_t bluetooth_manager_start_health_check(uint8_t check_items, uint32_t check_id, uint8_t group_id)
{
    broadcast_command_t cmd = {0};
    cmd.cmd_type = BROADCAST_CMD_HEALTH_CHECK;
    cmd.group_id = group_id;
    cmd.priority = PRIORITY_HIGH;
    cmd.params.health_check_params.check_items = check_items;
    cmd.params.health_check_params.check_id = check_id;
    
    ESP_LOGI(TAG, "Starting health check ID=%lu, items=0x%02X, group=%d", 
             check_id, check_items, group_id);
    
    return bluetooth_manager_broadcast_command(&cmd, NULL);
}

bt_mgr_err_t bluetooth_manager_broadcast_all_off(void)
{
    broadcast_command_t cmd = {0};
    cmd.cmd_type = BROADCAST_CMD_ALL_OFF;
    cmd.group_id = GROUP_ALL;
    cmd.priority = PRIORITY_HIGH;
    cmd.params.led_params.color = LED_COLOR_OFF;
    
    return bluetooth_manager_broadcast_command(&cmd, NULL);
}

bt_mgr_err_t bluetooth_manager_broadcast_beep(uint8_t group_id, uint8_t beep_mode)
{
    broadcast_command_t cmd = {0};
    cmd.cmd_type = BROADCAST_CMD_SET_BEEP;
    cmd.group_id = group_id;
    cmd.priority = PRIORITY_NORMAL;
    cmd.params.led_params.beep_mode = beep_mode;
    
    return bluetooth_manager_broadcast_command(&cmd, NULL);
}

/* =================== 分组管理接口 =================== */

bt_mgr_err_t bluetooth_manager_register_group(uint8_t group_id, const char *group_name, uint32_t estimated_count)
{
    if (!group_name || g_active_group_count >= MAX_DEVICE_GROUPS) {
        return BT_MGR_ERR_INVALID_PARAM;
    }
    
    xSemaphoreTake(g_manager_mutex, portMAX_DELAY);
    
    // 查找空闲槽位或现有分组
    device_group_t *group = find_group_by_id(group_id);
    if (!group) {
        // 找到空闲槽位
        for (int i = 0; i < MAX_DEVICE_GROUPS; i++) {
            if (g_device_groups[i].group_id == 0) {
                group = &g_device_groups[i];
                g_active_group_count++;
                break;
            }
        }
    }
    
    if (!group) {
        xSemaphoreGive(g_manager_mutex);
        return BT_MGR_ERR_GROUP_NOT_FOUND;
    }
    
    // 设置分组信息
    group->group_id = group_id;
    strncpy(group->group_name, group_name, sizeof(group->group_name) - 1);
    group->group_name[sizeof(group->group_name) - 1] = '\0';
    group->estimated_device_count = estimated_count;
    group->is_active = true;
    group->last_command_time = 0;
    
    xSemaphoreGive(g_manager_mutex);
    
    ESP_LOGI(TAG, "Registered group %d: %s (%lu devices)", group_id, group_name, estimated_count);
    return BT_MGR_OK;
}

const device_group_t* bluetooth_manager_get_group(uint8_t group_id)
{
    xSemaphoreTake(g_manager_mutex, portMAX_DELAY);
    device_group_t *group = find_group_by_id(group_id);
    xSemaphoreGive(g_manager_mutex);
    return group;
}

int bluetooth_manager_get_all_groups(device_group_t *groups, int max_count)
{
    if (!groups || max_count <= 0) {
        return 0;
    }
    
    int count = 0;
    xSemaphoreTake(g_manager_mutex, portMAX_DELAY);
    
    for (int i = 0; i < MAX_DEVICE_GROUPS && count < max_count; i++) {
        if (g_device_groups[i].group_id != 0) {
            memcpy(&groups[count], &g_device_groups[i], sizeof(device_group_t));
            count++;
        }
    }
    
    xSemaphoreGive(g_manager_mutex);
    return count;
}

bt_mgr_err_t bluetooth_manager_activate_group(uint8_t group_id)
{
    xSemaphoreTake(g_manager_mutex, portMAX_DELAY);
    device_group_t *group = find_group_by_id(group_id);
    
    if (!group) {
        xSemaphoreGive(g_manager_mutex);
        return BT_MGR_ERR_GROUP_NOT_FOUND;
    }
    
    group->is_active = true;
    xSemaphoreGive(g_manager_mutex);
    
    return BT_MGR_OK;
}

bt_mgr_err_t bluetooth_manager_deactivate_group(uint8_t group_id)
{
    xSemaphoreTake(g_manager_mutex, portMAX_DELAY);
    device_group_t *group = find_group_by_id(group_id);
    
    if (!group) {
        xSemaphoreGive(g_manager_mutex);
        return BT_MGR_ERR_GROUP_NOT_FOUND;
    }
    
    group->is_active = false;
    xSemaphoreGive(g_manager_mutex);
    
    return BT_MGR_OK;
}

/* =================== 高级控制接口 =================== */

int bluetooth_manager_activate_multiple_tags(const uint32_t *package_ids, int count, uint8_t color, bool enable_beep)
{
    if (!package_ids || count <= 0 || count > 4) {
        return 0;
    }
    
    broadcast_command_t cmd = {0};
    cmd.cmd_type = BROADCAST_CMD_GROUP_ACTIVATE;
    cmd.group_id = GROUP_ALL; // 可以根据需要调整为特定分组
    cmd.priority = PRIORITY_HIGH;
    cmd.params.led_params.color = color;
    cmd.params.led_params.beep_mode = enable_beep ? BEEP_MODE_SINGLE : BEEP_MODE_OFF;
    
    // 在目标参数中保存包裹ID
    for (int i = 0; i < count && i < 4; i++) {
        cmd.params.target_params.tag_ids[i] = package_ids[i];
    }
    cmd.params.target_params.count = count;
    
    bt_mgr_err_t result = bluetooth_manager_broadcast_command(&cmd, NULL);
    return (result == BT_MGR_OK) ? count : 0;
}

bt_mgr_err_t bluetooth_manager_express_station_mode(void)
{
    return bluetooth_manager_broadcast_all_set_color(LED_COLOR_BLUE, BLINK_MODE_NONE, 0);
}

bt_mgr_err_t bluetooth_manager_package_pickup_alert(uint8_t group_id, uint8_t alert_color)
{
    return bluetooth_manager_broadcast_group_set_color(group_id, alert_color, BLINK_MODE_SLOW, 30000); // 30秒提醒
}

bt_mgr_err_t bluetooth_manager_night_mode(void)
{
    return bluetooth_manager_broadcast_all_set_color(LED_COLOR_WHITE, BLINK_MODE_NONE, 0); // 夜间模式用白色低亮度
}

/* =================== 配置和统计接口 =================== */

bt_mgr_err_t bluetooth_manager_set_broadcast_interval(uint32_t interval_ms)
{
    if (interval_ms < 50 || interval_ms > 5000) {
        return BT_MGR_ERR_INVALID_PARAM;
    }
    
    g_broadcast_interval_ms = interval_ms;
    return BT_MGR_OK;
}

bt_mgr_err_t bluetooth_manager_set_repeat_count(uint8_t repeat_count)
{
    if (repeat_count > 10) {
        return BT_MGR_ERR_INVALID_PARAM;
    }
    
    g_repeat_count = repeat_count;
    return BT_MGR_OK;
}

const broadcast_stats_t* bluetooth_manager_get_stats(void)
{
    return &g_broadcast_stats;
}

bt_mgr_err_t bluetooth_manager_reset_stats(void)
{
    xSemaphoreTake(g_manager_mutex, portMAX_DELAY);
    memset(&g_broadcast_stats, 0, sizeof(g_broadcast_stats));
    g_broadcast_stats.current_sequence = 1;
    xSemaphoreGive(g_manager_mutex);
    
    return BT_MGR_OK;
}

void bluetooth_manager_set_broadcast_callback(broadcast_sent_cb_t cb)
{
    g_broadcast_sent_cb = cb;
}

void bluetooth_manager_set_feedback_callback(device_feedback_cb_t cb)
{
    g_device_feedback_cb = cb;
}

void bluetooth_manager_set_inventory_callback(inventory_result_cb_t cb)
{
    g_inventory_result_cb = cb;
}

void bluetooth_manager_set_health_check_callback(health_check_result_cb_t cb)
{
    g_health_check_result_cb = cb;
}

void bluetooth_manager_set_fault_report_callback(fault_report_cb_t cb)
{
    g_fault_report_cb = cb;
}

/* =================== 内部函数实现 =================== */

static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void on_reset(int reason)
{
    ESP_LOGI(TAG, "NimBLE stack reset, reason: %d", reason);
    g_manager_state = BT_MGR_STATE_INITIALIZED;
}

static void on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE stack synced");
    
    // 确保有正确的身份地址
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to ensure address: %d", rc);
        return;
    }
    
    g_manager_state = BT_MGR_STATE_READY;
    ESP_LOGI(TAG, "Bluetooth Broadcast Manager ready for operation");
}

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGD(TAG, "Advertising complete, reason: %d", event->adv_complete.reason);
            g_broadcast_state = BROADCAST_STATE_IDLE;
            
            // 如果需要重复发送
            if (g_current_repeat_count < g_repeat_count) {
                g_current_repeat_count++;
                ESP_LOGD(TAG, "Repeat broadcast %d/%d", g_current_repeat_count, g_repeat_count);
                
                // 启动重复定时器
                if (g_repeat_timer) {
                    esp_timer_start_once(g_repeat_timer, BLE_ADV_REPEAT_INTERVAL * 1000);
                }
            } else {
                // 广播完成
                g_broadcast_stats.successful_broadcasts++;
                if (g_broadcast_sent_cb) {
                    g_broadcast_sent_cb(BT_MGR_OK, g_current_packet.sequence);
                }
            }
            break;
            
        default:
            break;
    }
    
    return 0;
}

static void broadcast_timer_callback(void* arg)
{
    ESP_LOGD(TAG, "Broadcast timer callback");
    // 可以用于定期广播状态同步等
}

static void repeat_timer_callback(void* arg)
{
    ESP_LOGD(TAG, "Repeat timer callback");
    
    // 重新发送当前数据包
    if (g_broadcast_state == BROADCAST_STATE_IDLE) {
        start_advertising_packet(&g_current_packet);
    }
}

static bt_mgr_err_t start_advertising_packet(const broadcast_packet_t *packet)
{
    if (!packet) {
        ESP_LOGE(TAG, "Invalid packet pointer");
        return BT_MGR_ERR_BROADCAST_FAILED;
    }
    
    // 强制停止当前广播（如果有的话）
    ble_gap_adv_stop();
    g_broadcast_state = BROADCAST_STATE_IDLE;
    
    // 短暂延时确保广播完全停止
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 准备广播数据
    uint8_t adv_data[31];
    uint16_t adv_len = pack_broadcast_packet(packet, adv_data, sizeof(adv_data));
    
    if (adv_len == 0 || adv_len > 31) {
        ESP_LOGE(TAG, "Invalid advertising data length: %d", adv_len);
        return BT_MGR_ERR_PACKET_TOO_LARGE;
    }
    
    // 设置广播参数
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = (g_broadcast_interval_ms * 1000) / 625; // 转换为BLE时间单位
    adv_params.itvl_max = (g_broadcast_interval_ms * 1000) / 625;
    adv_params.channel_map = 0;
    adv_params.filter_policy = 0;
    adv_params.high_duty_cycle = 0;
    
    // 开始广播
    int rc = ble_gap_adv_set_data(adv_data, adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising data: %d", rc);
        return BT_MGR_ERR_BROADCAST_FAILED;
    }
    
    // 使用有限时间的广播而不是永久广播
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, 1000, // 1秒广播时间
                          &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
        g_broadcast_state = BROADCAST_STATE_IDLE;
        return BT_MGR_ERR_BROADCAST_FAILED;
    }
    
    g_broadcast_state = BROADCAST_STATE_ADVERTISING;
    ESP_LOGI(TAG, "Started advertising packet, length: %d, duration: 1000ms", adv_len);
    
    return BT_MGR_OK;
}

static bt_mgr_err_t stop_advertising(void)
{
    int rc = ble_gap_adv_stop();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Failed to stop advertising: %d", rc);
        return BT_MGR_ERR_BROADCAST_FAILED;
    }
    
    g_broadcast_state = BROADCAST_STATE_IDLE;
    return BT_MGR_OK;
}

static device_group_t* find_group_by_id(uint8_t group_id)
{
    for (int i = 0; i < MAX_DEVICE_GROUPS; i++) {
        if (g_device_groups[i].group_id == group_id) {
            return &g_device_groups[i];
        }
    }
    return NULL;
}

static void init_default_groups(void)
{
    // 初始化默认分组 - 快递驿站场景
    bluetooth_manager_register_group(GROUP_ALL, "All Devices", 5000);
    bluetooth_manager_register_group(GROUP_SHELF_A, "Shelf A", 1000);
    bluetooth_manager_register_group(GROUP_SHELF_B, "Shelf B", 1000);
    bluetooth_manager_register_group(GROUP_SHELF_C, "Shelf C", 1000);
    bluetooth_manager_register_group(GROUP_SHELF_D, "Shelf D", 1000);
    bluetooth_manager_register_group(GROUP_COUNTER, "Counter Area", 200);
    bluetooth_manager_register_group(GROUP_ENTRANCE, "Entrance Area", 100);
    bluetooth_manager_register_group(GROUP_SPECIAL, "Special Packages", 50);
    bluetooth_manager_register_group(GROUP_MAINTENANCE, "Maintenance", 10);
    
    ESP_LOGI(TAG, "Initialized %d express station device groups", g_active_group_count);
} 