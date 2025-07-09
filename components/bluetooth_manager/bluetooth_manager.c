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
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

static const char* TAG = "ESP32_Station_Central";

/* 外部函数声明 */
void ble_store_config_init(void);

/* 灯条设备服务和特征值UUID (16位) */
#define LED_STRIP_SERVICE_UUID_16           0x00FF
#define LED_STRIP_CMD_CHAR_UUID_16          0xFF01
#define LED_STRIP_NOTIFY_CHAR_UUID_16       0xFF02

/* 灯条设备识别 */
#define LED_DEVICE_NAME_PREFIX              "SmartTag"

/* 全局变量 */
static bt_mgr_state_t g_manager_state = BT_MGR_STATE_UNINITIALIZED;
static led_device_t g_devices[MAX_LED_DEVICES];
static SemaphoreHandle_t g_devices_mutex = NULL;
static bool g_is_scanning = false;
static device_found_cb_t g_device_found_cb = NULL;
static device_state_cb_t g_device_state_cb = NULL;
static device_response_cb_t g_device_response_cb = NULL;
static esp_timer_handle_t g_scan_timer = NULL;
static uint32_t g_scan_device_count = 0;  // 扫描到的设备计数

/* 内部函数声明 */
static void blecent_on_reset(int reason);
static void blecent_on_sync(void);
static int blecent_gap_event(struct ble_gap_event *event, void *arg);
static void blecent_scan(void);
static int blecent_should_connect(const struct ble_gap_disc_desc *disc);
static void blecent_connect_if_interesting(void *disc);
static void blecent_host_task(void *param);
static void scan_timeout_handler(void* arg);
static led_device_t* find_device_by_id(uint8_t device_id);
static led_device_t* find_device_by_conn_handle(uint16_t conn_handle);
static led_device_t* find_device_by_mac(const uint8_t *mac_addr);
static led_device_t* allocate_device_slot(void);
static void clear_device_slot(led_device_t *device);
static int blecent_on_gatt_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error,
                                    const struct ble_gatt_svc *service, void *arg);
static int blecent_on_gatt_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                                    const struct ble_gatt_chr *chr, void *arg);
static void debug_parse_adv_data(const uint8_t *data, uint8_t len);

/* 初始化蓝牙管理器 */
bt_mgr_err_t bluetooth_manager_init(void)
{
    esp_err_t ret;
    
    if (g_manager_state != BT_MGR_STATE_UNINITIALIZED) {
        ESP_LOGW(TAG, "Bluetooth manager already initialized");
        return BT_MGR_OK;
    }
    
    ESP_LOGI(TAG, "Initializing Bluetooth Station Manager (Central Mode)");
    
    // 创建设备列表互斥锁
    g_devices_mutex = xSemaphoreCreateMutex();
    if (!g_devices_mutex) {
        ESP_LOGE(TAG, "Failed to create devices mutex");
        return BT_MGR_ERR_INIT_FAILED;
    }
    
    // 初始化设备列表
    memset(g_devices, 0, sizeof(g_devices));
    for (int i = 0; i < MAX_LED_DEVICES; i++) {
        g_devices[i].device_id = 0;  // 0表示未使用
        g_devices[i].state = LED_DEVICE_STATE_DISCONNECTED;
        g_devices[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }
    
    // 初始化NimBLE
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize nimble port: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_devices_mutex);
        return BT_MGR_ERR_INIT_FAILED;
    }
    
    // 配置host回调函数
    ble_hs_cfg.reset_cb = blecent_on_reset;
    ble_hs_cfg.sync_cb = blecent_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    
    // 设置设备名称
    int rc = ble_svc_gap_device_name_set("ESP32_Station_Central");
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set device name: %d", rc);
        vSemaphoreDelete(g_devices_mutex);
        return BT_MGR_ERR_INIT_FAILED;
    }
    
    // 初始化存储配置 (暂时注释掉)
    // ble_store_config_init();
    
    g_manager_state = BT_MGR_STATE_INITIALIZED;
    ESP_LOGI(TAG, "Bluetooth Station Manager initialized successfully");
    
    return BT_MGR_OK;
}

/* 启动基站服务 */
bt_mgr_err_t bluetooth_manager_start(void)
{
    if (g_manager_state != BT_MGR_STATE_INITIALIZED) {
        ESP_LOGE(TAG, "Bluetooth manager not initialized");
        return BT_MGR_ERR_NOT_INITIALIZED;
    }
    
    ESP_LOGI(TAG, "Starting Bluetooth Station Manager");
    
    // 启动NimBLE host任务
    nimble_port_freertos_init(blecent_host_task);
    
    return BT_MGR_OK;
}

/* 停止基站服务 */
bt_mgr_err_t bluetooth_manager_stop(void)
{
    ESP_LOGI(TAG, "Stopping Bluetooth Station Manager");
    
    // 停止扫描
    bluetooth_manager_stop_scan();
    
    // 断开所有设备
    xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_LED_DEVICES; i++) {
        if (g_devices[i].device_id != 0 && g_devices[i].state == LED_DEVICE_STATE_CONNECTED) {
            ble_gap_terminate(g_devices[i].conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            clear_device_slot(&g_devices[i]);
        }
    }
    xSemaphoreGive(g_devices_mutex);
    
    // 停止NimBLE
    int rc = nimble_port_stop();
    if (rc == 0) {
        nimble_port_deinit();
    }
    
    g_manager_state = BT_MGR_STATE_INITIALIZED;
    return BT_MGR_OK;
}

/* 获取管理器状态 */
bt_mgr_state_t bluetooth_manager_get_state(void)
{
    return g_manager_state;
}

/* 开始扫描灯条设备 */
bt_mgr_err_t bluetooth_manager_start_scan(uint32_t duration_ms, device_found_cb_t found_cb)
{
    if (g_manager_state != BT_MGR_STATE_READY) {
        ESP_LOGE(TAG, "Bluetooth manager not ready for scanning");
        return BT_MGR_ERR_NOT_INITIALIZED;
    }
    
    if (g_is_scanning) {
        ESP_LOGW(TAG, "Already scanning");
        return BT_MGR_OK;
    }
    
    ESP_LOGI(TAG, "Starting BLE scan for Smart Tags (duration: %lu ms)", duration_ms);
    
    g_device_found_cb = found_cb;
    g_is_scanning = true;
    g_manager_state = BT_MGR_STATE_SCANNING;
    
    // 开始扫描
    blecent_scan();
    
    // 设置扫描超时定时器
    if (duration_ms > 0) {
        esp_timer_create_args_t timer_args = {
            .callback = scan_timeout_handler,
            .name = "scan_timeout"
        };
        
        if (g_scan_timer == NULL) {
            esp_timer_create(&timer_args, &g_scan_timer);
        }
        esp_timer_start_once(g_scan_timer, duration_ms * 1000);
    }
    
    ESP_LOGI(TAG, "BLE scan started");
    return BT_MGR_OK;
}

/* 停止扫描 */
bt_mgr_err_t bluetooth_manager_stop_scan(void)
{
    if (!g_is_scanning) {
        return BT_MGR_OK;
    }
    
    ESP_LOGI(TAG, "Stopping BLE scan");
    
    ble_gap_disc_cancel();
    g_is_scanning = false;
    g_device_found_cb = NULL;
    g_manager_state = BT_MGR_STATE_READY;
    
    // 停止扫描定时器
    if (g_scan_timer) {
        esp_timer_stop(g_scan_timer);
    }
    
    ESP_LOGI(TAG, "BLE scan stopped");
    return BT_MGR_OK;
}

/* 连接灯条设备 */
bt_mgr_err_t bluetooth_manager_connect_device(const uint8_t *mac_addr, uint8_t device_id)
{
    return bluetooth_manager_connect_device_with_type(mac_addr, device_id, BLE_ADDR_PUBLIC);
}

/* 连接灯条设备（带地址类型） */
bt_mgr_err_t bluetooth_manager_connect_device_with_type(const uint8_t *mac_addr, uint8_t device_id, uint8_t addr_type)
{
    if (!mac_addr || device_id == 0 || device_id > MAX_LED_DEVICES) {
        return BT_MGR_ERR_INVALID_PARAM;
    }
    
    // 检查当前管理器状态，避免重复连接操作
    if (g_manager_state == BT_MGR_STATE_CONNECTING) {
        ESP_LOGW(TAG, "Connection already in progress, rejecting new connection request");
        return BT_MGR_ERR_CONNECT_FAILED;
    }
    
    // 检查BLE Host是否已同步
    if (!ble_hs_synced()) {
        ESP_LOGE(TAG, "BLE Host not synced, cannot initiate connection");
        return BT_MGR_ERR_CONNECT_FAILED;
    }
    
    xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
    
    // 检查设备ID是否已被占用
    led_device_t *existing = find_device_by_id(device_id);
    if (existing && existing->state != LED_DEVICE_STATE_DISCONNECTED) {
        xSemaphoreGive(g_devices_mutex);
        ESP_LOGE(TAG, "Device ID %d already in use", device_id);
        return BT_MGR_ERR_DEVICE_FULL;
    }
    
    // 检查是否已有相同MAC地址的设备，如果有则清理
    for (int i = 0; i < MAX_LED_DEVICES; i++) {
        if (g_devices[i].device_id != 0 && 
            memcmp(g_devices[i].mac_addr, mac_addr, 6) == 0) {
            
            if (g_devices[i].state != LED_DEVICE_STATE_DISCONNECTED) {
                ESP_LOGW(TAG, "Found existing device with same MAC, cleaning up...");
                if (g_devices[i].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                    ble_gap_terminate(g_devices[i].conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }
                clear_device_slot(&g_devices[i]);
            }
        }
    }
    
    // 分配设备槽位
    led_device_t *device = allocate_device_slot();
    if (!device) {
        xSemaphoreGive(g_devices_mutex);
        ESP_LOGE(TAG, "No available device slots");
        return BT_MGR_ERR_DEVICE_FULL;
    }
    
    // 初始化设备信息
    device->device_id = device_id;
    memcpy(device->mac_addr, mac_addr, 6);
    device->addr_type = addr_type;  // 保存地址类型
    device->state = LED_DEVICE_STATE_CONNECTING;
    device->conn_handle = BLE_HS_CONN_HANDLE_NONE;
    device->last_seen = esp_timer_get_time() / 1000;
    
    xSemaphoreGive(g_devices_mutex);
    
    ESP_LOGI(TAG, "Connecting to device ID %d, MAC: %02x:%02x:%02x:%02x:%02x:%02x, addr_type: %d", 
             device_id, mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], addr_type);
    
    // 转换MAC地址格式，使用正确的地址类型
    ble_addr_t ble_addr;
    ble_addr.type = addr_type;  // 使用传入的地址类型而不是硬编码
    memcpy(ble_addr.val, mac_addr, 6);
    
    // 停止扫描以便连接，并等待扫描完全停止
    if (g_is_scanning) {
        ESP_LOGI(TAG, "Stopping scan for connection attempt");
        ble_gap_disc_cancel();
        g_is_scanning = false;
        
        // 等待扫描停止（最多100ms）
        int wait_count = 0;
        while (ble_gap_disc_active() && wait_count < 10) {
            vTaskDelay(pdMS_TO_TICKS(10));
            wait_count++;
        }
        
        if (ble_gap_disc_active()) {
            ESP_LOGW(TAG, "Scan still active after timeout, proceeding anyway");
        } else {
            ESP_LOGI(TAG, "Scan stopped successfully");
        }
    }
    
    // 额外等待确保BLE栈状态稳定
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 检查是否有其他正在进行的连接
    if (ble_gap_conn_active()) {
        ESP_LOGW(TAG, "Another connection is active, this might cause issues");
        ESP_LOGW(TAG, "Attempting to cancel any active connection operations...");
        
        // 强制取消所有可能的连接操作
        ble_gap_conn_cancel();
        
        // 等待连接操作完全取消
        int cancel_wait = 0;
        while (ble_gap_conn_active() && cancel_wait < 20) {
            vTaskDelay(pdMS_TO_TICKS(10));
            cancel_wait++;
        }
        
        if (ble_gap_conn_active()) {
            ESP_LOGE(TAG, "Failed to cancel active connection, aborting");
            xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
            clear_device_slot(device);
            xSemaphoreGive(g_devices_mutex);
            g_manager_state = BT_MGR_STATE_READY;
            return BT_MGR_ERR_CONNECT_FAILED;
        } else {
            ESP_LOGI(TAG, "Successfully cancelled active connection operations");
        }
    }
    
    // 获取own address type
    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining own address type: %d", rc);
        xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
        clear_device_slot(device);
        xSemaphoreGive(g_devices_mutex);
        return BT_MGR_ERR_CONNECT_FAILED;
    }
    
    ESP_LOGI(TAG, "Connection parameters: own_addr_type=%d, peer_addr_type=%d, timeout=%d ms", 
             own_addr_type, addr_type, BLE_CONNECT_TIMEOUT_MS);
    
    // 设置连接状态
    g_manager_state = BT_MGR_STATE_CONNECTING;
    
    // 开始连接，使用保守的连接参数以提高成功率
    struct ble_gap_conn_params conn_params = {
        .scan_itvl = 0x20,      // 20ms (更保守的扫描间隔)
        .scan_window = 0x10,    // 10ms  
        .itvl_min = 40,         // 50ms (更宽松的连接间隔)
        .itvl_max = 80,         // 100ms
        .latency = 0,
        .supervision_timeout = 400,  // 4s (更长的超时时间)
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    
    ESP_LOGI(TAG, "Attempting connection with scan_itvl=%d, scan_window=%d, conn_itvl=%d-%d", 
             conn_params.scan_itvl, conn_params.scan_window, conn_params.itvl_min, conn_params.itvl_max);
    
    rc = ble_gap_connect(own_addr_type, &ble_addr, BLE_CONNECT_TIMEOUT_MS, 
                        &conn_params, blecent_gap_event, device);
    
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to initiate connection: %d (BLE error code)", rc);
        
        // 根据错误码提供具体的原因分析
        switch (rc) {
            case 2: // BLE_HS_EALREADY
                ESP_LOGE(TAG, "Connection already in progress or device busy");
                ESP_LOGE(TAG, "This suggests the target device may be in an inconsistent state");
                ESP_LOGE(TAG, "Recommendation: Wait longer before retrying, or reset target device");
                break;
            case 3: // BLE_HS_EINVAL
                ESP_LOGE(TAG, "Invalid connection parameters");
                break;
            case 15: // BLE_HS_EBUSY
                ESP_LOGE(TAG, "BLE stack is busy, try again later");
                break;
    default:
                ESP_LOGE(TAG, "Other BLE error, check device availability");
                ESP_LOGE(TAG, "Possible causes: device out of range, interference, or device-side issues");
        break;
    }

        xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
        clear_device_slot(device);
        xSemaphoreGive(g_devices_mutex);
        g_manager_state = BT_MGR_STATE_READY;
        return BT_MGR_ERR_CONNECT_FAILED;
    }
    
    ESP_LOGI(TAG, "Connection request sent successfully, waiting for result...");
    return BT_MGR_OK;
}

/* 断开灯条设备 */
bt_mgr_err_t bluetooth_manager_disconnect_device(uint8_t device_id)
{
    xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
    
    led_device_t *device = find_device_by_id(device_id);
    if (!device || device->state != LED_DEVICE_STATE_CONNECTED) {
        xSemaphoreGive(g_devices_mutex);
        ESP_LOGE(TAG, "Device ID %d not found or not connected", device_id);
        return BT_MGR_ERR_DEVICE_NOT_FOUND;
    }
    
    uint16_t conn_handle = device->conn_handle;
    xSemaphoreGive(g_devices_mutex);
    
    ESP_LOGI(TAG, "Disconnecting device ID %d", device_id);
    
    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to disconnect device: %d", rc);
        return BT_MGR_ERR_CONNECT_FAILED;
    }
    
    return BT_MGR_OK;
}

/* 发送命令到指定灯条 */
bt_mgr_err_t bluetooth_manager_send_command(uint8_t device_id, const led_command_t *cmd)
{
    if (!cmd || device_id == 0 || device_id > MAX_LED_DEVICES) {
        ESP_LOGE(TAG, "Invalid parameters: device_id=%d, cmd=%p", device_id, cmd);
        return BT_MGR_ERR_INVALID_PARAM;
    }
    
    xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
    
    led_device_t *device = find_device_by_id(device_id);
    if (!device) {
        xSemaphoreGive(g_devices_mutex);
        ESP_LOGE(TAG, "Device ID %d not found in device list", device_id);
        return BT_MGR_ERR_DEVICE_NOT_FOUND;
    }
    
    if (device->state != LED_DEVICE_STATE_CONNECTED) {
        xSemaphoreGive(g_devices_mutex);
        ESP_LOGE(TAG, "Device ID %d not connected (state: %d)", device_id, device->state);
        return BT_MGR_ERR_DEVICE_NOT_FOUND;
    }
    
    uint16_t conn_handle = device->conn_handle;
    uint16_t cmd_handle = device->cmd_chr_handle;
    ESP_LOGI(TAG, "Device ID %d connection details: handle=%d, chr_handle=%d", 
             device_id, conn_handle, cmd_handle);
    xSemaphoreGive(g_devices_mutex);
    
    // 检查GATT句柄是否有效
    if (cmd_handle == 0) {
        ESP_LOGE(TAG, "Device ID %d: Invalid characteristic handle (service discovery may have failed)", device_id);
        return BT_MGR_ERR_CONNECT_FAILED;
    }
    
    // 使用协议帧格式发送命令
    uint8_t frame_buf[40];
    uint8_t cmd_data[7];  // 根据协议文档，命令数据7字节：命令类型(1) + 数据长度(1) + 设备ID(2) + 颜色(1) + 持续时间(2)
    
    // 根据协议文档构建命令数据
    cmd_data[0] = CMD_SET_LIGHT_COLOR;        // 命令类型：设置亮灯颜色
    cmd_data[1] = 0x05;                         // 数据长度：5字节
    cmd_data[2] = (device_id >> 8) & 0xFF;     // 设备ID高字节
    cmd_data[3] = device_id & 0xFF;            // 设备ID低字节
    cmd_data[4] = cmd->param1 & 0xFF;          // 颜色值
    // 持续时间转换为秒（大端格式）
    uint16_t duration_sec = (cmd->param2 + 999) / 1000;  // 毫秒转秒，向上取整
    cmd_data[5] = (duration_sec >> 8) & 0xFF;  // 持续时间高字节
    cmd_data[6] = duration_sec & 0xFF;         // 持续时间低字节
    
    ESP_LOGI(TAG, "Command data prepared");
    
    // 使用协议帧打包函数
    uint16_t frame_len = protocol_frame_pack(frame_buf, cmd_data, sizeof(cmd_data));
    if (frame_len == 0) {
        ESP_LOGE(TAG, "Failed to pack protocol frame for device ID %d", device_id);
        return BT_MGR_ERR_INVALID_PARAM;
    }
    ESP_LOGI(TAG, "Protocol frame packed, length: %d", frame_len);
    
    ESP_LOGI(TAG, "Sending protocol frame to device ID %d: %d bytes", device_id, frame_len);
    ESP_LOG_BUFFER_HEX(TAG, frame_buf, frame_len);
    
    // 通过GATT发送协议帧
    int rc = ble_gattc_write_no_rsp_flat(conn_handle, cmd_handle, frame_buf, frame_len);
    
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send protocol frame to device ID %d: error=%d", device_id, rc);
        
        // 检查连接是否还有效
        if (rc == BLE_HS_ENOTCONN) {
            ESP_LOGE(TAG, "Connection lost for device ID %d", device_id);
        } else if (rc == BLE_HS_EINVAL) {
            ESP_LOGE(TAG, "Invalid handle or data for device ID %d", device_id);
        }
        
        return BT_MGR_ERR_CONNECT_FAILED;
    }
    
    ESP_LOGI(TAG, "Protocol frame sent successfully to device ID %d", device_id);
    return BT_MGR_OK;
}

/* 广播命令到所有已连接灯条 */
int bluetooth_manager_broadcast_command(const led_command_t *cmd)
{
    if (!cmd) {
        return 0;
    }
    
    int success_count = 0;
    
    xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
    
    for (int i = 0; i < MAX_LED_DEVICES; i++) {
        if (g_devices[i].device_id != 0 && g_devices[i].state == LED_DEVICE_STATE_CONNECTED) {
            uint8_t device_id = g_devices[i].device_id;
            xSemaphoreGive(g_devices_mutex);
            
            if (bluetooth_manager_send_command(device_id, cmd) == BT_MGR_OK) {
                success_count++;
            }
            
            xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
        }
    }
    
    xSemaphoreGive(g_devices_mutex);
    
    ESP_LOGI(TAG, "Broadcast command sent to %d devices", success_count);
    return success_count;
}

/* 获取设备信息 */
const led_device_t* bluetooth_manager_get_device(uint8_t device_id)
{
    xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
    led_device_t *device = find_device_by_id(device_id);
    xSemaphoreGive(g_devices_mutex);
    return device;
}

/* 获取所有已连接设备列表 */
int bluetooth_manager_get_connected_devices(led_device_t *devices, int max_count)
{
    if (!devices || max_count <= 0) {
        return 0;
    }
    
    int count = 0;
    
    xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
    
    for (int i = 0; i < MAX_LED_DEVICES && count < max_count; i++) {
        if (g_devices[i].device_id != 0 && g_devices[i].state == LED_DEVICE_STATE_CONNECTED) {
            memcpy(&devices[count], &g_devices[i], sizeof(led_device_t));
            count++;
        }
    }
    
    xSemaphoreGive(g_devices_mutex);
    
    return count;
}

/* 获取已连接设备数量 */
int bluetooth_manager_get_connected_count(void)
{
    int count = 0;
    
    xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
    
    for (int i = 0; i < MAX_LED_DEVICES; i++) {
        if (g_devices[i].device_id != 0 && g_devices[i].state == LED_DEVICE_STATE_CONNECTED) {
            count++;
        }
    }
    
    xSemaphoreGive(g_devices_mutex);
    
    return count;
}

/* 设置设备状态变化回调 */
void bluetooth_manager_set_state_callback(device_state_cb_t cb)
{
    g_device_state_cb = cb;
}

/* 设置设备响应回调 */
void bluetooth_manager_set_response_callback(device_response_cb_t cb)
{
    g_device_response_cb = cb;
}

/* 检查设备是否在线 */
bool bluetooth_manager_is_device_online(uint8_t device_id)
{
    xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
    led_device_t *device = find_device_by_id(device_id);
    bool online = (device && device->state == LED_DEVICE_STATE_CONNECTED);
    xSemaphoreGive(g_devices_mutex);
    return online;
}

/* 获取设备信号强度 */
int8_t bluetooth_manager_get_device_rssi(uint8_t device_id)
{
    xSemaphoreTake(g_devices_mutex, portMAX_DELAY);
    led_device_t *device = find_device_by_id(device_id);
    int8_t rssi = device ? device->rssi : -128;
    xSemaphoreGive(g_devices_mutex);
    return rssi;
}

/* ========== 内部函数实现 ========== */

/* NimBLE栈重置回调 */
static void blecent_on_reset(int reason)
{
    ESP_LOGI(TAG, "NimBLE stack reset, reason: %d", reason);
    g_manager_state = BT_MGR_STATE_INITIALIZED;
}

/* NimBLE栈同步回调 */
static void blecent_on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE stack synced");
    
    // 确保有正确的身份地址
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to ensure address: %d", rc);
        return;
    }
    
    g_manager_state = BT_MGR_STATE_READY;
    ESP_LOGI(TAG, "Bluetooth Central ready for operation");
}

/* BLE host任务 */
static void blecent_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    
    // 这个函数会一直运行直到nimble_port_stop()被调用
    nimble_port_run();
    
    nimble_port_freertos_deinit();
}

/* 开始扫描 */
static void blecent_scan(void)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    ESP_LOGI(TAG, "[SCAN] Starting BLE device scan...");

    // 重置扫描计数器
    g_scan_device_count = 0;
    
    // 获取地址类型
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining address type: %d", rc);
        return;
    }

    // 配置扫描参数 - 优化参数以提高发现率
    disc_params.filter_duplicates = 0;  // 不过滤重复设备，确保能发现所有广播
    disc_params.passive = 1;            // 被动扫描，不发送scan request
    disc_params.itvl = 0x0010;         // 扫描间隔 16*0.625ms = 10ms
    disc_params.window = 0x0010;       // 扫描窗口 16*0.625ms = 10ms (100%占空比)
    disc_params.filter_policy = 0;      // 接受所有广播包
    disc_params.limited = 0;            // 一般可发现模式
    
    ESP_LOGI(TAG, "[SCAN] Scan params - interval: %d, window: %d, passive: %d, filter_dup: %d", 
             disc_params.itvl, disc_params.window, disc_params.passive, disc_params.filter_duplicates);
    
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                      blecent_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error initiating GAP discovery procedure: %d", rc);
    } else {
        ESP_LOGI(TAG, "[SCAN] BLE discovery started successfully, looking for SmartTag devices...");
    }
}

/* 判断是否应该连接到设备 */
static int blecent_should_connect(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;
    int rc;
    
    // 设备必须是可连接的
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
        ESP_LOGD(TAG, "[FILTER] Device not connectable, event_type: %d", disc->event_type);
        return 0;
    }
    
    // 解析广播数据
    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0) {
        ESP_LOGW(TAG, "[FILTER] Failed to parse adv fields: %d", rc);
        return 0;
    }
    
    // 提取设备名称用于调试
    char device_name[32] = "UNKNOWN";
    if (fields.name != NULL && fields.name_len > 0) {
        int copy_len = fields.name_len < 31 ? fields.name_len : 31;
        memcpy(device_name, fields.name, copy_len);
        device_name[copy_len] = '\0';
    }
    
    ESP_LOGI(TAG, "[FILTER] Checking device: %s", device_name);
    
    // 检查设备名称 - 检查是否以"SmartTag"开头
    if (fields.name != NULL && fields.name_len > 0) {
        ESP_LOGI(TAG, "[FILTER] Name check: comparing '%s' with prefix '%s'", device_name, LED_DEVICE_NAME_PREFIX);
        if (strncmp((char*)fields.name, LED_DEVICE_NAME_PREFIX, strlen(LED_DEVICE_NAME_PREFIX)) == 0) {
            ESP_LOGI(TAG, "[FILTER] ✓ Name match! Device '%s' matches prefix '%s'", device_name, LED_DEVICE_NAME_PREFIX);
            return 1;
        } else {
            ESP_LOGI(TAG, "[FILTER] ✗ Name mismatch: '%s' does not start with '%s'", device_name, LED_DEVICE_NAME_PREFIX);
        }
    } else {
        ESP_LOGI(TAG, "[FILTER] ✗ No device name in advertisement");
    }
    
    // 检查是否广播了我们的服务UUID
    ESP_LOGI(TAG, "[FILTER] Checking %d advertised UUIDs for service 0x%04X:", fields.num_uuids16, LED_STRIP_SERVICE_UUID_16);
    for (int i = 0; i < fields.num_uuids16; i++) {
        uint16_t uuid = ble_uuid_u16(&fields.uuids16[i].u);
        ESP_LOGI(TAG, "[FILTER] UUID %d: 0x%04X", i, uuid);
        if (uuid == LED_STRIP_SERVICE_UUID_16) {
            ESP_LOGI(TAG, "[FILTER] ✓ Service UUID match! Found target service 0x%04X", LED_STRIP_SERVICE_UUID_16);
            return 1;
        }
    }
    
    if (fields.num_uuids16 == 0) {
        ESP_LOGI(TAG, "[FILTER] ✗ No service UUIDs advertised");
    } else {
        ESP_LOGI(TAG, "[FILTER] ✗ Target service UUID 0x%04X not found", LED_STRIP_SERVICE_UUID_16);
    }
    
    return 0;
}

/* 如果设备有趣则连接 */
static void blecent_connect_if_interesting(void *disc)
{
    struct ble_gap_disc_desc *disc_desc = (struct ble_gap_disc_desc *)disc;
    struct ble_hs_adv_fields fields;
    int rc;

    // 解析广播数据
    rc = ble_hs_adv_parse_fields(&fields, disc_desc->data, disc_desc->length_data);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to parse advertisement fields: %d, data_len: %d", rc, disc_desc->length_data);
        
        // 即使解析失败，也尝试显示原始数据用于调试
        if (disc_desc->length_data > 0) {
            ESP_LOGW(TAG, "Raw advertisement data:");
            ESP_LOG_BUFFER_HEX(TAG, disc_desc->data, disc_desc->length_data > 31 ? 31 : disc_desc->length_data);
            
            // 尝试手动解析广播数据
            debug_parse_adv_data(disc_desc->data, disc_desc->length_data);
        }
        
        // 对于解析失败的设备，仍然尝试回调，但标记为未知设备
        if (g_device_found_cb) {
            scan_result_t result = {0};
            memcpy(result.mac_addr, disc_desc->addr.val, 6);
            result.addr_type = disc_desc->addr.type;  // 传递地址类型
            result.rssi = disc_desc->rssi;
            strcpy(result.name, "PARSE_ERROR");
            result.is_led_device = false;
            
            char addr_str[18];
            sprintf(addr_str, "%02x:%02x:%02x:%02x:%02x:%02x",
                    disc_desc->addr.val[0], disc_desc->addr.val[1], disc_desc->addr.val[2],
                    disc_desc->addr.val[3], disc_desc->addr.val[4], disc_desc->addr.val[5]);
            
            ESP_LOGW(TAG, "[SCAN_ERROR] Parse failed for device: %s RSSI: %d", addr_str, disc_desc->rssi);
            g_device_found_cb(&result);
        }
        return;
    }
    
    // 记录所有发现的设备（用于调试）
    char addr_str[18];
    sprintf(addr_str, "%02x:%02x:%02x:%02x:%02x:%02x",
            disc_desc->addr.val[0], disc_desc->addr.val[1], disc_desc->addr.val[2],
            disc_desc->addr.val[3], disc_desc->addr.val[4], disc_desc->addr.val[5]);
    
    char name_str[32] = "UNKNOWN";
    if (fields.name != NULL && fields.name_len > 0) {
        int copy_len = fields.name_len < 31 ? fields.name_len : 31;
        memcpy(name_str, fields.name, copy_len);
        name_str[copy_len] = '\0';
    }
    
    g_scan_device_count++;
    ESP_LOGI(TAG, "[SCAN_RAW] Found device #%lu: %s (%s) RSSI: %d, addr_type: %d, event_type: %d", 
             g_scan_device_count, name_str, addr_str, disc_desc->rssi, disc_desc->addr.type, disc_desc->event_type);
    
    // 检查服务UUID
    for (int i = 0; i < fields.num_uuids16; i++) {
        uint16_t uuid = ble_uuid_u16(&fields.uuids16[i].u);
        ESP_LOGI(TAG, "[SCAN_SVC] Device %s advertises service UUID: 0x%04X", name_str, uuid);
    }
    
    // 如果回调函数存在，通知发现的设备
    if (g_device_found_cb) {
        scan_result_t result = {0};
        memcpy(result.mac_addr, disc_desc->addr.val, 6);
        result.addr_type = disc_desc->addr.type;  // 传递地址类型
        result.rssi = disc_desc->rssi;
        
        // 正确设置设备名称
        if (fields.name != NULL && fields.name_len > 0) {
            int copy_len = fields.name_len < sizeof(result.name) - 1 ? fields.name_len : sizeof(result.name) - 1;
            memcpy(result.name, fields.name, copy_len);
            result.name[copy_len] = '\0';
        } else {
            strcpy(result.name, "UNKNOWN");
        }
        
        // 判断是否为目标LED设备
        result.is_led_device = blecent_should_connect(disc_desc);
        
        ESP_LOGI(TAG, "[SCAN_RESULT] Device: %s, MAC: %s, RSSI: %d, addr_type: %d, IsTarget: %s", 
                 result.name, addr_str, result.rssi, result.addr_type, result.is_led_device ? "YES" : "NO");
        
        g_device_found_cb(&result);
    }
}

/* GAP事件回调函数 */
static int blecent_gap_event(struct ble_gap_event *event, void *arg)
{
    ESP_LOGI(TAG, "GAP event: type=%d", event->type);
    
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        // 设备发现事件
        blecent_connect_if_interesting(&event->disc);
        return 0;
        
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "GAP Connect event, conn_handle: %d, status: %d", 
                event->connect.conn_handle, event->connect.status);
        ESP_LOGI(TAG, "[CONNECT_DEBUG] Event arg pointer: %p", arg);
        
        if (event->connect.status == 0) {
            // 连接成功事件 - 但需要验证连接是否真正可用
            led_device_t *device = (led_device_t*)arg;
            ESP_LOGI(TAG, "[CONNECT_DEBUG] Device pointer: %p", device);
            
            // 验证连接是否真正建立
            struct ble_gap_conn_desc conn_desc;
            int conn_check = ble_gap_conn_find(event->connect.conn_handle, &conn_desc);
            if (conn_check != 0) {
                ESP_LOGE(TAG, "[CONNECT_VERIFY] Connection verification failed: handle %d not found", event->connect.conn_handle);
                ESP_LOGE(TAG, "[CONNECT_VERIFY] This indicates a false connection success");
                
                if (device) {
                    // 不使用互斥锁，直接设置状态
                    device->state = LED_DEVICE_STATE_ERROR;
                    device->conn_handle = BLE_HS_CONN_HANDLE_NONE;
                    device->device_id = 0;  // 标记为未使用
                }
                g_manager_state = BT_MGR_STATE_READY;
                return 0;
            }
            
            ESP_LOGI(TAG, "[CONNECT_VERIFY] Connection verified successfully");
            ESP_LOGI(TAG, "[CONNECT_VERIFY] Peer address: %02x:%02x:%02x:%02x:%02x:%02x, type: %d",
                     conn_desc.peer_id_addr.val[0], conn_desc.peer_id_addr.val[1], 
                     conn_desc.peer_id_addr.val[2], conn_desc.peer_id_addr.val[3],
                     conn_desc.peer_id_addr.val[4], conn_desc.peer_id_addr.val[5],
                     conn_desc.peer_id_addr.type);
            
            if (device) {
                ESP_LOGI(TAG, "[CONNECT_DEBUG] Device ID: %d", device->device_id);
                
                // 不使用互斥锁，直接设置状态
                device->conn_handle = event->connect.conn_handle;
                device->state = LED_DEVICE_STATE_CONNECTING;
                
                // 调用状态变化回调
                if (g_device_state_cb) {
                    g_device_state_cb(device->device_id, LED_DEVICE_STATE_DISCONNECTED, LED_DEVICE_STATE_CONNECTING);
                }
                
                ESP_LOGI(TAG, "Device ID %d connection established, starting service discovery...", device->device_id);
                
                // 等待连接稳定
                vTaskDelay(pdMS_TO_TICKS(100));
                
                // 再次验证连接
                if (ble_gap_conn_find(event->connect.conn_handle, &conn_desc) != 0) {
                    ESP_LOGE(TAG, "[CONNECT_VERIFY] Connection lost during stabilization");
                    device->state = LED_DEVICE_STATE_ERROR;
                    device->conn_handle = BLE_HS_CONN_HANDLE_NONE;
                    device->device_id = 0;  // 标记为未使用
                    g_manager_state = BT_MGR_STATE_READY;
                    return 0;
                }
                
                // 开始服务发现
                ESP_LOGI(TAG, "Starting service discovery for device ID %d...", device->device_id);
                int svc_rc = ble_gattc_disc_all_svcs(event->connect.conn_handle, blecent_on_gatt_disc_svc, device);
                if (svc_rc != 0) {
                    ESP_LOGE(TAG, "Failed to start service discovery: %d", svc_rc);
                    ESP_LOGE(TAG, "[CONNECT_DEBUG] Service discovery failed - connection may be unstable");
                    device->state = LED_DEVICE_STATE_ERROR;
                    device->conn_handle = BLE_HS_CONN_HANDLE_NONE;
                    device->device_id = 0;  // 标记为未使用
                    
                    // 强制断开连接
                    ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                } else {
                    ESP_LOGI(TAG, "Service discovery initiated successfully");
                }
            } else {
                ESP_LOGE(TAG, "[CONNECT_DEBUG] Device pointer is NULL! Cannot proceed with service discovery");
                ESP_LOGE(TAG, "[CONNECT_DEBUG] This indicates a problem with the connection setup");
                // 尝试找到设备通过conn_handle
                led_device_t *found_device = find_device_by_conn_handle(event->connect.conn_handle);
                if (found_device) {
                    ESP_LOGI(TAG, "[CONNECT_DEBUG] Found device by conn_handle: ID %d", found_device->device_id);
                    found_device->state = LED_DEVICE_STATE_CONNECTING;
                    
                    // 开始服务发现
                    ESP_LOGI(TAG, "Starting service discovery for recovered device ID %d...", found_device->device_id);
                    int svc_rc = ble_gattc_disc_all_svcs(event->connect.conn_handle, blecent_on_gatt_disc_svc, found_device);
                    if (svc_rc != 0) {
                        ESP_LOGE(TAG, "Failed to start service discovery: %d", svc_rc);
                        found_device->state = LED_DEVICE_STATE_ERROR;
                        found_device->conn_handle = BLE_HS_CONN_HANDLE_NONE;
                        found_device->device_id = 0;  // 标记为未使用
                    } else {
                        ESP_LOGI(TAG, "Service discovery initiated successfully for recovered device");
                    }
                } else {
                    ESP_LOGE(TAG, "[CONNECT_DEBUG] Could not find device by conn_handle either");
                    // 强制断开这个无效的连接
                    ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }
            }
            g_manager_state = BT_MGR_STATE_READY;
        } else {
            // 连接失败
            ESP_LOGE(TAG, "Connection failed with status: %d", event->connect.status);
            led_device_t *device = (led_device_t*)arg;
            if (device) {
                // 调用状态变化回调
                if (g_device_state_cb) {
                    g_device_state_cb(device->device_id, device->state, LED_DEVICE_STATE_ERROR);
                }
                
                device->state = LED_DEVICE_STATE_ERROR;
                device->conn_handle = BLE_HS_CONN_HANDLE_NONE;
                device->device_id = 0;  // 标记为未使用
            }
            g_manager_state = BT_MGR_STATE_READY;
        }
        break;
        
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "GAP Disconnect event, conn_handle: %d, reason: %d", 
                event->disconnect.conn.conn_handle, event->disconnect.reason);
        
        led_device_t *device = find_device_by_conn_handle(event->disconnect.conn.conn_handle);
        if (device) {
            ESP_LOGI(TAG, "Device ID %d disconnected", device->device_id);
            
            // 调用状态变化回调
            if (g_device_state_cb) {
                g_device_state_cb(device->device_id, device->state, LED_DEVICE_STATE_DISCONNECTED);
            }
            
            device->state = LED_DEVICE_STATE_DISCONNECTED;
            device->conn_handle = BLE_HS_CONN_HANDLE_NONE;
            device->device_id = 0;  // 标记为未使用
        }
        break;
        
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "[SCAN] BLE scan completed - reason: %d", event->disc_complete.reason);
        ESP_LOGI(TAG, "[SCAN] Total devices found: %lu", g_scan_device_count);
        ESP_LOGI(TAG, "[SCAN] Scan finished, switching back to READY state");
        g_is_scanning = false;
        g_manager_state = BT_MGR_STATE_READY;
        break;
        
    case BLE_GAP_EVENT_NOTIFY_RX:
        // 收到通知
        ESP_LOGI(TAG, "Received notification from conn_handle=%d attr_handle=%d",
                event->notify_rx.conn_handle, event->notify_rx.attr_handle);
        
        if (g_device_response_cb) {
            led_device_t *device = find_device_by_conn_handle(event->notify_rx.conn_handle);
            if (device) {
                // 提取通知数据
                uint8_t data[256];
                uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
                if (len > sizeof(data)) len = sizeof(data);
                
                int copy_result = os_mbuf_copydata(event->notify_rx.om, 0, len, data);
                if (copy_result == 0) {
                    ESP_LOGI(TAG, "✅ Notification data copied successfully: %d bytes", len);
                    ESP_LOG_BUFFER_HEX(TAG, data, len);
                    g_device_response_cb(device->device_id, data, len);
                } else {
                    ESP_LOGW(TAG, "Failed to copy notification data: %d", copy_result);
                }
            } else {
                ESP_LOGW(TAG, "Device not found for conn_handle: %d", event->notify_rx.conn_handle);
            }
        } else {
            ESP_LOGW(TAG, "No response callback registered");
        }
        return 0;
        
    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "GAP Connection parameters updated, conn_handle: %d, status: %d",
                event->conn_update.conn_handle, event->conn_update.status);
        break;
        
    default:
        ESP_LOGD(TAG, "Unhandled GAP event: %d", event->type);
        break;
    }
    
    return 0;
}

/* GATT特征值发现回调（简化版） */
static int blecent_on_gatt_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                                    const struct ble_gatt_chr *chr, void *arg)
{
    led_device_t *device = (led_device_t*)arg;
    
    if (error->status != 0) {
        ESP_LOGE(TAG, "Characteristic discovery failed: status=%d conn_handle=%d", 
                error->status, conn_handle);
        return 0;
    }
    
    if (chr == NULL) {
        ESP_LOGI(TAG, "Characteristic discovery complete: conn_handle=%d", conn_handle);
        ESP_LOGI(TAG, "All characteristics discovered for device ID %d", device ? device->device_id : 0);
        
        // 检查是否已经发现了必要的特征值
        if (device && device->cmd_chr_handle != 0 && device->notify_chr_handle != 0) {
            ESP_LOGI(TAG, "[SERVICE_DISCOVERY] Device ID %d service discovery completed successfully", device->device_id);
            ESP_LOGI(TAG, "[SERVICE_DISCOVERY] Command handle: %d, Notify handle: %d", 
                     device->cmd_chr_handle, device->notify_chr_handle);
            
            // 如果设备已经连接，则不再进行后续操作
            if (device->state == LED_DEVICE_STATE_CONNECTED) {
                ESP_LOGI(TAG, "[SERVICE_DISCOVERY] Device ID %d already connected, skipping further operations", device->device_id);
                return 0;
            }
            
            // 标记为已连接
            device->state = LED_DEVICE_STATE_CONNECTED;
            ESP_LOGI(TAG, "[SERVICE_DISCOVERY] Device ID %d now marked as CONNECTED", device->device_id);
        } else {
            ESP_LOGI(TAG, "[SERVICE_DISCOVERY] Characteristic discovery completed but missing required characteristics");
            ESP_LOGI(TAG, "[SERVICE_DISCOVERY] cmd_handle: %d, notify_handle: %d", 
                     device ? device->cmd_chr_handle : -1, device ? device->notify_chr_handle : -1);
        }
        return 0;
    }
    
    // 打印发现的特征值信息
    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&chr->uuid.u, uuid_str);
    ESP_LOGI(TAG, "Discovered characteristic: UUID=%s, handle=%d, properties=0x%02x", 
             uuid_str, chr->val_handle, chr->properties);
    
    // 只处理三个目标特征值
    bool is_cmd_char = false;
    bool is_notify_char = false;
    
    // 检查下发特征值 (0xFF01)
    if (ble_uuid_cmp(&chr->uuid.u, BLE_UUID16_DECLARE(LED_STRIP_CMD_CHAR_UUID_16)) == 0) {
        is_cmd_char = true;
        ESP_LOGI(TAG, "Found command characteristic (0xFF01) for device ID %d", device ? device->device_id : 0);
    }
    
    // 检查接收特征值 (0xFF02)  
    if (ble_uuid_cmp(&chr->uuid.u, BLE_UUID16_DECLARE(LED_STRIP_NOTIFY_CHAR_UUID_16)) == 0) {
        is_notify_char = true;
        ESP_LOGI(TAG, "Found notify characteristic (0xFF02) for device ID %d", device ? device->device_id : 0);
    }
    
    if (is_cmd_char && device) {
        device->cmd_chr_handle = chr->val_handle;
        ESP_LOGI(TAG, "✅ Command characteristic handle set: %d", device->cmd_chr_handle);
    } else if (is_notify_char && device) {
        device->notify_chr_handle = chr->val_handle;
        ESP_LOGI(TAG, "✅ Notify characteristic handle set: %d", device->notify_chr_handle);
        
        // 找到通知特征值后，如果命令特征值也已找到，则标记为连接成功
        if (device->cmd_chr_handle != 0) {
            ESP_LOGI(TAG, "[DISCOVERY_SUCCESS] Found both characteristics! Marking device as CONNECTED");
            
            // 调用状态变化回调
            if (g_device_state_cb) {
                g_device_state_cb(device->device_id, device->state, LED_DEVICE_STATE_CONNECTED);
            }
            
            device->state = LED_DEVICE_STATE_CONNECTED;
            ESP_LOGI(TAG, "[DISCOVERY_SUCCESS] Device ID %d marked as CONNECTED", device->device_id);
            
            // 对于一对多场景，只停止当前连接的特征值发现
            ESP_LOGI(TAG, "[DISCOVERY_SUCCESS] Device ID %d discovery completed successfully", device->device_id);
            
            return 1;  // 停止当前连接的特征值发现过程
        }
    } else {
        ESP_LOGI(TAG, "Ignoring non-target characteristic: %s", uuid_str);
    }
    
    return 0;
}

/* GATT服务发现回调（简化版） */
static int blecent_on_gatt_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error,
                                    const struct ble_gatt_svc *service, void *arg)
{
    led_device_t *device = (led_device_t*)arg;
    
    if (error->status != 0) {
        ESP_LOGE(TAG, "Service discovery failed: status=%d conn_handle=%d", 
                error->status, conn_handle);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    
    if (service == NULL) {
        ESP_LOGI(TAG, "Service discovery complete: conn_handle=%d", conn_handle);
        ESP_LOGI(TAG, "All services have been discovered for device ID %d", device ? device->device_id : 0);
        return 0;
    }
    
    // 打印发现的服务信息
    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&service->uuid.u, uuid_str);
    ESP_LOGI(TAG, "Discovered service: UUID=%s, start_handle=%d, end_handle=%d", 
             uuid_str, service->start_handle, service->end_handle);
    
    // 检查是否已经连接成功(避免重复发现)
    if (device && device->state == LED_DEVICE_STATE_CONNECTED) {
        ESP_LOGI(TAG, "Device already connected, skipping service discovery for: %s", uuid_str);
        return 0;
    }
    
    // 只处理 0x00FF 服务，完全忽略其他服务
    if (ble_uuid_cmp(&service->uuid.u, BLE_UUID16_DECLARE(LED_STRIP_SERVICE_UUID_16)) == 0) {
        ESP_LOGI(TAG, "✅ Found target service (0x00FF) for device ID %d", device ? device->device_id : 0);
        ESP_LOGI(TAG, "🔍 Starting characteristic discovery for service UUID: %s", uuid_str);
        ESP_LOGI(TAG, "Service handle range: %d - %d", 
                service->start_handle, service->end_handle);
        
        // 开始发现该服务的特征值
        int chr_rc = ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle,
                                            blecent_on_gatt_disc_chr, device);
        if (chr_rc != 0) {
            ESP_LOGE(TAG, "Failed to start characteristic discovery: %d", chr_rc);
        } else {
            ESP_LOGI(TAG, "✅ Characteristic discovery initiated successfully");
        }
        
        // 找到目标服务后，停止当前连接的服务发现过程，不影响其他设备
        ESP_LOGI(TAG, "✅ Target service found for device ID %d, stopping service discovery for this connection", 
                 device ? device->device_id : 0);
        return 1;  // 停止当前连接的服务发现过程
    } else {
        ESP_LOGI(TAG, "Ignoring non-target service: %s", uuid_str);
    }
    
    return 0;
}

/* 扫描超时处理 */
static void scan_timeout_handler(void* arg)
{
    ESP_LOGI(TAG, "Scan timeout, stopping scan");
    bluetooth_manager_stop_scan();
}

/* 根据设备ID查找设备 */
static led_device_t* find_device_by_id(uint8_t device_id)
{
    for (int i = 0; i < MAX_LED_DEVICES; i++) {
        if (g_devices[i].device_id == device_id) {
            return &g_devices[i];
        }
    }
    return NULL;
}

/* 根据连接句柄查找设备 */
static led_device_t* find_device_by_conn_handle(uint16_t conn_handle)
{
    for (int i = 0; i < MAX_LED_DEVICES; i++) {
        if (g_devices[i].conn_handle == conn_handle) {
            return &g_devices[i];
        }
    }
    return NULL;
}

/* 根据MAC地址查找设备 (暂时未使用，保留供将来扩展) */
static led_device_t* find_device_by_mac(const uint8_t *mac_addr) __attribute__((unused));
static led_device_t* find_device_by_mac(const uint8_t *mac_addr)
{
    for (int i = 0; i < MAX_LED_DEVICES; i++) {
        if (memcmp(g_devices[i].mac_addr, mac_addr, 6) == 0) {
            return &g_devices[i];
        }
    }
    return NULL;
}

/* 分配设备槽位 */
static led_device_t* allocate_device_slot(void)
{
    for (int i = 0; i < MAX_LED_DEVICES; i++) {
        if (g_devices[i].device_id == 0) {
            return &g_devices[i];
        }
    }
    return NULL;
}

/* 清空设备槽位 */
static void clear_device_slot(led_device_t *device)
{
    if (!device) return;
    
    memset(device, 0, sizeof(led_device_t));
    device->device_id = 0;
    device->state = LED_DEVICE_STATE_DISCONNECTED;
    device->conn_handle = BLE_HS_CONN_HANDLE_NONE;
}

/* 手动解析广播数据用于调试 */
static void debug_parse_adv_data(const uint8_t *data, uint8_t len)
{
    if (!data || len == 0) {
        ESP_LOGW(TAG, "[DEBUG] No advertisement data to parse");
        return;
    }
    
    ESP_LOGI(TAG, "[DEBUG] Manually parsing advertisement data (%d bytes):", len);
    
    uint8_t pos = 0;
    while (pos < len) {
        if (pos + 1 >= len) break;
        
        uint8_t field_len = data[pos];
        uint8_t field_type = data[pos + 1];
        
        if (field_len == 0 || pos + field_len + 1 > len) {
            ESP_LOGW(TAG, "[DEBUG] Invalid field length %d at pos %d", field_len, pos);
            break;
        }
        
        ESP_LOGI(TAG, "[DEBUG] Field type: 0x%02X, length: %d", field_type, field_len);
        
        switch (field_type) {
            case 0x01: // Flags
                if (field_len >= 3) {
                    ESP_LOGI(TAG, "[DEBUG] Flags: 0x%02X", data[pos + 2]);
                }
                break;
                
            case 0x02: // Partial list of 16-bit UUIDs
            case 0x03: // Complete list of 16-bit UUIDs
                ESP_LOGI(TAG, "[DEBUG] 16-bit UUIDs (%s):", 
                         field_type == 0x02 ? "partial" : "complete");
                for (int i = 0; i < field_len - 1; i += 2) {
                    if (pos + 3 + i < len) {
                        uint16_t uuid = data[pos + 2 + i] | (data[pos + 3 + i] << 8);
                        ESP_LOGI(TAG, "[DEBUG]   UUID: 0x%04X", uuid);
                        
                        // 检查是否为目标服务UUID
                        if (uuid == LED_STRIP_SERVICE_UUID_16) {
                            ESP_LOGI(TAG, "[DEBUG]   *** TARGET SERVICE FOUND! ***");
                        }
                    }
                }
                break;
                
            case 0x08: // Shortened local name
            case 0x09: // Complete local name
                {
                    char name[33] = {0};
                    int name_len = field_len - 1;
                    if (name_len > 32) name_len = 32;
                    memcpy(name, &data[pos + 2], name_len);
                    ESP_LOGI(TAG, "[DEBUG] Device name (%s): '%s'", 
                             field_type == 0x08 ? "shortened" : "complete", name);
                    
                    // 检查是否为目标设备名称
                    if (strncmp(name, LED_DEVICE_NAME_PREFIX, strlen(LED_DEVICE_NAME_PREFIX)) == 0) {
                        ESP_LOGI(TAG, "[DEBUG]   *** TARGET DEVICE NAME FOUND! ***");
                    }
                }
                break;
                
            case 0x0A: // Tx Power Level
                if (field_len >= 3) {
                    int8_t tx_power = (int8_t)data[pos + 2];
                    ESP_LOGI(TAG, "[DEBUG] Tx Power: %d dBm", tx_power);
                }
                break;
                
            case 0x0D: // Class of Device
                if (field_len >= 5) {
                    uint32_t cod = data[pos + 2] | (data[pos + 3] << 8) | (data[pos + 4] << 16);
                    ESP_LOGI(TAG, "[DEBUG] Class of Device: 0x%06" PRIX32, cod);
                }
                break;
                
            case 0x16: // Service Data - 16-bit UUID
                if (field_len >= 5) {
                    uint16_t uuid = data[pos + 2] | (data[pos + 3] << 8);
                    ESP_LOGI(TAG, "[DEBUG] Service Data for UUID 0x%04X:", uuid);
                    ESP_LOG_BUFFER_HEX(TAG, &data[pos + 4], field_len - 3);
                }
                break;
                
            case 0xFF: // Manufacturer Specific Data
                if (field_len >= 5) {
                    uint16_t company_id = data[pos + 2] | (data[pos + 3] << 8);
                    ESP_LOGI(TAG, "[DEBUG] Manufacturer Data (Company: 0x%04X):", company_id);
                    ESP_LOG_BUFFER_HEX(TAG, &data[pos + 4], field_len - 3);
                }
                break;
                
            default:
                ESP_LOGI(TAG, "[DEBUG] Unknown field type 0x%02X, data:", field_type);
                ESP_LOG_BUFFER_HEX(TAG, &data[pos + 2], field_len - 1);
                break;
        }
        
        pos += field_len + 1;
    }
} 

/* 协议帧打包函数（简化版） */
uint16_t protocol_frame_pack(uint8_t *frame_buf, const uint8_t *data, uint16_t data_len)
{
    if (!frame_buf || data_len > 32) {
        return 0;
    }
    
    frame_buf[0] = 0xAA;                    // 帧头1
    frame_buf[1] = 0x55;                    // 帧头2
    
    // 复制数据（如果有数据）
    if (data && data_len > 0) {
        memcpy(&frame_buf[2], data, data_len);
    }
    
    frame_buf[2 + data_len] = 0x55;         // 帧尾1
    frame_buf[3 + data_len] = 0xAA;         // 帧尾2
    
    return 4 + data_len;                    // 返回总帧长度
}

/* 协议帧解包函数（简化版） */
uint16_t protocol_frame_unpack(const uint8_t *frame_buf, uint16_t frame_len, 
                              uint8_t *data_buf, uint16_t data_buf_size)
{
    if (!frame_buf || !data_buf || frame_len < 4) {
        return 0;
    }
    
    // 检查帧头
    if (frame_buf[0] != 0xAA || frame_buf[1] != 0x55) {
        return 0;  // 帧头错误
    }
    
    uint16_t data_len = frame_len - 4;  // 去除帧头和帧尾
    
    // 检查帧尾
    if (frame_buf[frame_len - 2] != 0x55 || frame_buf[frame_len - 1] != 0xAA) {
        return 0;  // 帧尾错误
    }
    
    // 检查数据长度
    if (data_len > data_buf_size) {
        return 0;  // 缓冲区不足
    }
    
    // 复制数据
    if (data_len > 0) {
        memcpy(data_buf, &frame_buf[2], data_len);
    }
    
    return data_len;
}

/* 创建响应帧函数 */
uint16_t create_response_frame(uint8_t cmd_type, uint16_t device_id, const uint8_t *data, 
                              uint8_t data_len, uint8_t *frame_buf, uint16_t frame_buf_size)
{
    if (!frame_buf || frame_buf_size < 5) {
        return 0;
    }
    
    // 创建响应数据结构
    strip_to_station_data_t response;
    response.cmd_type = cmd_type;
    response.device_id = device_id;  // 大端格式
    
    // 复制数据到响应结构
    uint8_t copy_len = data_len;
    if (copy_len > sizeof(response.data)) {
        copy_len = sizeof(response.data);
    }
    
    if (data && copy_len > 0) {
        memcpy(response.data, data, copy_len);
    }
    
    // 计算响应结构的总长度
    uint16_t response_len = 3 + copy_len;  // cmd_type(1) + device_id(2) + data
    
    // 使用协议帧打包函数打包响应
    return protocol_frame_pack(frame_buf, (uint8_t*)&response, response_len);
} 