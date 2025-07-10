#include "inventory_manager.h"
#include "bluetooth_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "INVENTORY_MGR";

/* 最大标签数量 */
#define MAX_TAGS 1000

/* 标签信息存储 */
typedef struct {
    inventory_info_t info;
    bool is_active;
    uint32_t last_update_time;
} tag_record_t;

/* 盘点管理器状态 */
typedef struct {
    inventory_state_t state;
    uint32_t current_scan_id;
    uint32_t scan_start_time;
    uint32_t scan_timeout_ms;
    uint8_t scan_group_id;
    uint8_t scan_type;
    
    // 统计信息
    inventory_stats_t stats;
    
    // 标签记录
    tag_record_t *tag_records;
    uint16_t max_tags;
    uint16_t active_tags;
    
    // 回调函数
    inventory_complete_cb_t complete_cb;
    tag_discovered_cb_t discovery_cb;
    
    // 同步
    SemaphoreHandle_t mutex;
    esp_timer_handle_t scan_timer;
    
    bool initialized;
} inventory_manager_t;

static inventory_manager_t g_inventory_mgr = {0};

/* 内部函数声明 */
static void inventory_scan_timeout_handler(void *arg);
static void inventory_complete_scan(inventory_state_t final_state);
static tag_record_t* find_tag_record(uint32_t tag_id);
static tag_record_t* allocate_tag_record(uint32_t tag_id);
static void update_scan_stats(void);

/* =================== 公共API实现 =================== */

inventory_err_t inventory_manager_init(void)
{
    if (g_inventory_mgr.initialized) {
        return INVENTORY_ERR_OK;
    }
    
    // 分配标签记录内存
    g_inventory_mgr.tag_records = calloc(MAX_TAGS, sizeof(tag_record_t));
    if (!g_inventory_mgr.tag_records) {
        ESP_LOGE(TAG, "Failed to allocate tag records");
        return INVENTORY_ERR_NO_MEMORY;
    }
    
    // 创建互斥锁
    g_inventory_mgr.mutex = xSemaphoreCreateMutex();
    if (!g_inventory_mgr.mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(g_inventory_mgr.tag_records);
        return INVENTORY_ERR_NO_MEMORY;
    }
    
    // 创建扫描定时器
    esp_timer_create_args_t timer_args = {
        .callback = inventory_scan_timeout_handler,
        .arg = NULL,
        .name = "inventory_scan_timer"
    };
    
    esp_err_t ret = esp_timer_create(&timer_args, &g_inventory_mgr.scan_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create scan timer: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_inventory_mgr.mutex);
        free(g_inventory_mgr.tag_records);
        return INVENTORY_ERR_NO_MEMORY;
    }
    
    // 初始化状态
    g_inventory_mgr.max_tags = MAX_TAGS;
    g_inventory_mgr.state = INVENTORY_STATE_IDLE;
    g_inventory_mgr.initialized = true;
    
    ESP_LOGI(TAG, "Inventory manager initialized, max tags: %d", MAX_TAGS);
    return INVENTORY_ERR_OK;
}

void inventory_manager_deinit(void)
{
    if (!g_inventory_mgr.initialized) {
        return;
    }
    
    // 停止当前扫描
    inventory_manager_stop_scan();
    
    // 清理资源
    if (g_inventory_mgr.scan_timer) {
        esp_timer_delete(g_inventory_mgr.scan_timer);
    }
    
    if (g_inventory_mgr.mutex) {
        vSemaphoreDelete(g_inventory_mgr.mutex);
    }
    
    if (g_inventory_mgr.tag_records) {
        free(g_inventory_mgr.tag_records);
    }
    
    memset(&g_inventory_mgr, 0, sizeof(g_inventory_mgr));
    ESP_LOGI(TAG, "Inventory manager deinitialized");
}

inventory_err_t inventory_manager_start_full_scan(uint32_t scan_id, uint32_t timeout_ms)
{
    if (!g_inventory_mgr.initialized) {
        return INVENTORY_ERR_NOT_INITIALIZED;
    }
    
    if (g_inventory_mgr.state != INVENTORY_STATE_IDLE) {
        return INVENTORY_ERR_SCAN_ACTIVE;
    }
    
    xSemaphoreTake(g_inventory_mgr.mutex, portMAX_DELAY);
    
    // 初始化扫描状态
    g_inventory_mgr.state = INVENTORY_STATE_SCANNING;
    g_inventory_mgr.current_scan_id = scan_id;
    g_inventory_mgr.scan_start_time = esp_timer_get_time() / 1000;
    g_inventory_mgr.scan_timeout_ms = timeout_ms;
    g_inventory_mgr.scan_group_id = GROUP_ALL;
    g_inventory_mgr.scan_type = 0; // 全部扫描
    
    // 重置统计信息
    memset(&g_inventory_mgr.stats, 0, sizeof(g_inventory_mgr.stats));
    
    // 启动超时定时器
    esp_timer_start_once(g_inventory_mgr.scan_timer, timeout_ms * 1000);
    
    xSemaphoreGive(g_inventory_mgr.mutex);
    
    // 发送盘点广播
    bt_mgr_err_t ret = bluetooth_manager_start_inventory_scan(scan_id, 0, GROUP_ALL, timeout_ms);
    if (ret != BT_MGR_OK) {
        ESP_LOGE(TAG, "Failed to start inventory broadcast: %d", ret);
        inventory_complete_scan(INVENTORY_STATE_ERROR);
        return INVENTORY_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "Started full inventory scan ID=%lu, timeout=%lums", scan_id, timeout_ms);
    return INVENTORY_ERR_OK;
}

inventory_err_t inventory_manager_start_group_scan(uint32_t scan_id, uint8_t group_id, uint32_t timeout_ms)
{
    if (!g_inventory_mgr.initialized) {
        return INVENTORY_ERR_NOT_INITIALIZED;
    }
    
    if (g_inventory_mgr.state != INVENTORY_STATE_IDLE) {
        return INVENTORY_ERR_SCAN_ACTIVE;
    }
    
    xSemaphoreTake(g_inventory_mgr.mutex, portMAX_DELAY);
    
    // 初始化扫描状态
    g_inventory_mgr.state = INVENTORY_STATE_SCANNING;
    g_inventory_mgr.current_scan_id = scan_id;
    g_inventory_mgr.scan_start_time = esp_timer_get_time() / 1000;
    g_inventory_mgr.scan_timeout_ms = timeout_ms;
    g_inventory_mgr.scan_group_id = group_id;
    g_inventory_mgr.scan_type = 1; // 分组扫描
    
    // 重置统计信息
    memset(&g_inventory_mgr.stats, 0, sizeof(g_inventory_mgr.stats));
    
    // 启动超时定时器
    esp_timer_start_once(g_inventory_mgr.scan_timer, timeout_ms * 1000);
    
    xSemaphoreGive(g_inventory_mgr.mutex);
    
    // 发送盘点广播
    bt_mgr_err_t ret = bluetooth_manager_start_inventory_scan(scan_id, 1, group_id, timeout_ms);
    if (ret != BT_MGR_OK) {
        ESP_LOGE(TAG, "Failed to start group inventory broadcast: %d", ret);
        inventory_complete_scan(INVENTORY_STATE_ERROR);
        return INVENTORY_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "Started group inventory scan ID=%lu, group=%d, timeout=%lums", scan_id, group_id, timeout_ms);
    return INVENTORY_ERR_OK;
}

inventory_err_t inventory_manager_stop_scan(void)
{
    if (!g_inventory_mgr.initialized) {
        return INVENTORY_ERR_NOT_INITIALIZED;
    }
    
    if (g_inventory_mgr.state == INVENTORY_STATE_IDLE) {
        return INVENTORY_ERR_OK;
    }
    
    // 停止定时器
    esp_timer_stop(g_inventory_mgr.scan_timer);
    
    // 完成扫描
    inventory_complete_scan(INVENTORY_STATE_COMPLETED);
    
    ESP_LOGI(TAG, "Inventory scan stopped");
    return INVENTORY_ERR_OK;
}

inventory_state_t inventory_manager_get_state(void)
{
    return g_inventory_mgr.state;
}

inventory_err_t inventory_manager_get_stats(inventory_stats_t *stats)
{
    if (!g_inventory_mgr.initialized || !stats) {
        return INVENTORY_ERR_INVALID_PARAM;
    }
    
    xSemaphoreTake(g_inventory_mgr.mutex, portMAX_DELAY);
    update_scan_stats();
    *stats = g_inventory_mgr.stats;
    xSemaphoreGive(g_inventory_mgr.mutex);
    
    return INVENTORY_ERR_OK;
}

void inventory_manager_set_complete_callback(inventory_complete_cb_t cb)
{
    g_inventory_mgr.complete_cb = cb;
}

void inventory_manager_set_discovery_callback(tag_discovered_cb_t cb)
{
    g_inventory_mgr.discovery_cb = cb;
}

inventory_err_t inventory_manager_handle_tag_response(uint32_t tag_id, const uint8_t *data, uint16_t length)
{
    if (!g_inventory_mgr.initialized || !data) {
        return INVENTORY_ERR_INVALID_PARAM;
    }
    
    if (g_inventory_mgr.state != INVENTORY_STATE_SCANNING && g_inventory_mgr.state != INVENTORY_STATE_COLLECTING) {
        return INVENTORY_ERR_OK; // 忽略非扫描状态的响应
    }
    
    xSemaphoreTake(g_inventory_mgr.mutex, portMAX_DELAY);
    
    // 查找或分配标签记录
    tag_record_t *record = find_tag_record(tag_id);
    if (!record) {
        record = allocate_tag_record(tag_id);
        if (!record) {
            xSemaphoreGive(g_inventory_mgr.mutex);
            return INVENTORY_ERR_NO_MEMORY;
        }
    }
    
    // 更新标签信息
    record->info.tag_id = tag_id;
    record->info.last_seen_time = esp_timer_get_time() / 1000;
    record->is_active = true;
    record->last_update_time = record->info.last_seen_time;
    
    // 解析响应数据 (简化版本，实际应根据协议解析)
    if (length >= 8) {
        record->info.battery_level = data[0];
        record->info.voltage_mv = (data[1] << 8) | data[2];
        record->info.led_status = data[3];
        record->info.beep_status = data[4];
        record->info.signal_strength = data[5];
        record->info.group_id = data[6];
        record->info.fault_flags = data[7];
    }
    
    xSemaphoreGive(g_inventory_mgr.mutex);
    
    // 触发发现回调
    if (g_inventory_mgr.discovery_cb) {
        g_inventory_mgr.discovery_cb(g_inventory_mgr.current_scan_id, &record->info);
    }
    
    ESP_LOGD(TAG, "Tag response: ID=%lu, battery=%d%%, voltage=%dmV", 
             tag_id, record->info.battery_level, record->info.voltage_mv);
    
    return INVENTORY_ERR_OK;
}

inventory_err_t inventory_manager_get_tag_info(uint32_t tag_id, inventory_info_t *info)
{
    if (!g_inventory_mgr.initialized || !info) {
        return INVENTORY_ERR_INVALID_PARAM;
    }
    
    xSemaphoreTake(g_inventory_mgr.mutex, portMAX_DELAY);
    
    tag_record_t *record = find_tag_record(tag_id);
    if (!record || !record->is_active) {
        xSemaphoreGive(g_inventory_mgr.mutex);
        return INVENTORY_ERR_INVALID_PARAM;
    }
    
    *info = record->info;
    xSemaphoreGive(g_inventory_mgr.mutex);
    
    return INVENTORY_ERR_OK;
}

inventory_err_t inventory_manager_clear_tag_info(uint32_t tag_id)
{
    if (!g_inventory_mgr.initialized) {
        return INVENTORY_ERR_NOT_INITIALIZED;
    }
    
    xSemaphoreTake(g_inventory_mgr.mutex, portMAX_DELAY);
    
    if (tag_id == 0) {
        // 清除所有标签
        for (uint16_t i = 0; i < g_inventory_mgr.max_tags; i++) {
            g_inventory_mgr.tag_records[i].is_active = false;
        }
        g_inventory_mgr.active_tags = 0;
        ESP_LOGI(TAG, "Cleared all tag records");
    } else {
        // 清除指定标签
        tag_record_t *record = find_tag_record(tag_id);
        if (record && record->is_active) {
            record->is_active = false;
            g_inventory_mgr.active_tags--;
            ESP_LOGD(TAG, "Cleared tag record: ID=%lu", tag_id);
        }
    }
    
    xSemaphoreGive(g_inventory_mgr.mutex);
    return INVENTORY_ERR_OK;
}

inventory_err_t inventory_manager_set_tag_fault(uint32_t tag_id, uint8_t fault_flags)
{
    if (!g_inventory_mgr.initialized) {
        return INVENTORY_ERR_NOT_INITIALIZED;
    }
    
    xSemaphoreTake(g_inventory_mgr.mutex, portMAX_DELAY);
    
    tag_record_t *record = find_tag_record(tag_id);
    if (!record) {
        record = allocate_tag_record(tag_id);
        if (!record) {
            xSemaphoreGive(g_inventory_mgr.mutex);
            return INVENTORY_ERR_NO_MEMORY;
        }
        record->info.tag_id = tag_id;
    }
    
    record->info.fault_flags = fault_flags;
    record->info.last_seen_time = esp_timer_get_time() / 1000;
    
    xSemaphoreGive(g_inventory_mgr.mutex);
    
    ESP_LOGI(TAG, "Set tag fault: ID=%lu, flags=0x%02X", tag_id, fault_flags);
    return INVENTORY_ERR_OK;
}

inventory_err_t inventory_manager_get_low_battery_tags(uint32_t *tag_ids, uint16_t max_count, uint16_t *actual_count)
{
    if (!g_inventory_mgr.initialized || !tag_ids || !actual_count) {
        return INVENTORY_ERR_INVALID_PARAM;
    }
    
    xSemaphoreTake(g_inventory_mgr.mutex, portMAX_DELAY);
    
    uint16_t count = 0;
    for (uint16_t i = 0; i < g_inventory_mgr.max_tags && count < max_count; i++) {
        tag_record_t *record = &g_inventory_mgr.tag_records[i];
        if (record->is_active && record->info.battery_level < 20) { // 低于20%为低电量
            tag_ids[count++] = record->info.tag_id;
        }
    }
    
    *actual_count = count;
    xSemaphoreGive(g_inventory_mgr.mutex);
    
    return INVENTORY_ERR_OK;
}

inventory_err_t inventory_manager_get_faulty_tags(uint32_t *tag_ids, uint16_t max_count, uint16_t *actual_count)
{
    if (!g_inventory_mgr.initialized || !tag_ids || !actual_count) {
        return INVENTORY_ERR_INVALID_PARAM;
    }
    
    xSemaphoreTake(g_inventory_mgr.mutex, portMAX_DELAY);
    
    uint16_t count = 0;
    for (uint16_t i = 0; i < g_inventory_mgr.max_tags && count < max_count; i++) {
        tag_record_t *record = &g_inventory_mgr.tag_records[i];
        if (record->is_active && record->info.fault_flags != 0) {
            tag_ids[count++] = record->info.tag_id;
        }
    }
    
    *actual_count = count;
    xSemaphoreGive(g_inventory_mgr.mutex);
    
    return INVENTORY_ERR_OK;
}

/* =================== 内部函数实现 =================== */

static void inventory_scan_timeout_handler(void *arg)
{
    ESP_LOGI(TAG, "Inventory scan timeout");
    inventory_complete_scan(INVENTORY_STATE_COMPLETED);
}

static void inventory_complete_scan(inventory_state_t final_state)
{
    if (g_inventory_mgr.state == INVENTORY_STATE_IDLE) {
        return;
    }
    
    xSemaphoreTake(g_inventory_mgr.mutex, portMAX_DELAY);
    
    // 更新最终统计信息
    update_scan_stats();
    g_inventory_mgr.stats.scan_duration_ms = (esp_timer_get_time() / 1000) - g_inventory_mgr.scan_start_time;
    
    // 保存统计信息
    inventory_stats_t stats = g_inventory_mgr.stats;
    uint32_t scan_id = g_inventory_mgr.current_scan_id;
    
    // 更新状态
    g_inventory_mgr.state = final_state;
    
    xSemaphoreGive(g_inventory_mgr.mutex);
    
    // 触发完成回调
    if (g_inventory_mgr.complete_cb) {
        g_inventory_mgr.complete_cb(scan_id, final_state, &stats);
    }
    
    ESP_LOGI(TAG, "Inventory scan completed: ID=%lu, state=%d, responding=%lu/%lu, duration=%lums",
             scan_id, final_state, stats.responding_tags, stats.total_tags, stats.scan_duration_ms);
    
    // 重置为空闲状态
    g_inventory_mgr.state = INVENTORY_STATE_IDLE;
}

static tag_record_t* find_tag_record(uint32_t tag_id)
{
    for (uint16_t i = 0; i < g_inventory_mgr.max_tags; i++) {
        tag_record_t *record = &g_inventory_mgr.tag_records[i];
        if (record->is_active && record->info.tag_id == tag_id) {
            return record;
        }
    }
    return NULL;
}

static tag_record_t* allocate_tag_record(uint32_t tag_id)
{
    // 查找空闲记录
    for (uint16_t i = 0; i < g_inventory_mgr.max_tags; i++) {
        tag_record_t *record = &g_inventory_mgr.tag_records[i];
        if (!record->is_active) {
            memset(record, 0, sizeof(tag_record_t));
            record->is_active = true;
            record->info.tag_id = tag_id;
            g_inventory_mgr.active_tags++;
            return record;
        }
    }
    
    ESP_LOGW(TAG, "No free tag record slots available");
    return NULL;
}

static void update_scan_stats(void)
{
    g_inventory_mgr.stats.total_tags = g_inventory_mgr.active_tags;
    g_inventory_mgr.stats.responding_tags = 0;
    g_inventory_mgr.stats.low_battery_tags = 0;
    g_inventory_mgr.stats.faulty_tags = 0;
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    uint32_t scan_window = g_inventory_mgr.scan_timeout_ms;
    
    for (uint16_t i = 0; i < g_inventory_mgr.max_tags; i++) {
        tag_record_t *record = &g_inventory_mgr.tag_records[i];
        if (!record->is_active) continue;
        
        // 检查是否在扫描窗口内响应
        if (current_time - record->info.last_seen_time <= scan_window) {
            g_inventory_mgr.stats.responding_tags++;
        }
        
        // 检查低电量
        if (record->info.battery_level < 20) {
            g_inventory_mgr.stats.low_battery_tags++;
        }
        
        // 检查故障
        if (record->info.fault_flags != 0) {
            g_inventory_mgr.stats.faulty_tags++;
        }
    }
}