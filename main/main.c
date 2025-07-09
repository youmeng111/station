/*
 * ESP32智能基站主程序
 * 基于ESP-IDF官方NimBLE模板重构
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"

/* 组件头文件 */
#include "system_config.h"
#include "led_controller.h"
#include "bluetooth_manager.h"
#include "mqtt_manager.h"
#include "ota_manager.h"

static const char *TAG = "MAIN";

/* 静态变量用于跟踪连接重试 */
static uint32_t g_last_connection_attempt = 0;
static uint8_t g_last_attempted_mac[6] = {0};
static const uint32_t CONNECTION_RETRY_INTERVAL_MS = 10000; // 10秒重试间隔

/* 设备发现回调函数 */
static void on_device_found(const scan_result_t *result) {
    ESP_LOGI(TAG, "[SCAN] ===== BLE SCAN RESULT =====");
    ESP_LOGI(TAG, "  [DEV] Device Name: %s", result->name);
    ESP_LOGI(TAG, "  [MAC] MAC Address: %02x:%02x:%02x:%02x:%02x:%02x", 
             result->mac_addr[0], result->mac_addr[1], result->mac_addr[2],
             result->mac_addr[3], result->mac_addr[4], result->mac_addr[5]);
    ESP_LOGI(TAG, "  [RSSI] Signal: %d dBm", result->rssi);
    ESP_LOGI(TAG, "  [TYPE] Is Smart Tag: %s", result->is_led_device ? "YES ^_^" : "NO -_-");
    
    // 如果是目标LED设备，可以尝试自动连接（测试用）
    if (result->is_led_device && strlen(result->name) > 0) {
        // 检查重试间隔，避免过于频繁的连接尝试
        uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒
        
        // 如果是同一设备且在重试间隔内，跳过此次连接
        if (memcmp(g_last_attempted_mac, result->mac_addr, 6) == 0 &&
            (current_time - g_last_connection_attempt) < CONNECTION_RETRY_INTERVAL_MS) {
            
            uint32_t remaining_time = CONNECTION_RETRY_INTERVAL_MS - (current_time - g_last_connection_attempt);
            ESP_LOGI(TAG, "[SKIP] Skipping connection attempt, retry in %lu ms", remaining_time);
            return;
        }
        
        ESP_LOGI(TAG, "[CONN] Found target Smart Tag! Attempting connection... (>_<)");
        
        // 记录连接尝试
        g_last_connection_attempt = current_time;
        memcpy(g_last_attempted_mac, result->mac_addr, 6);
        
        // 添加小延迟避免连接冲突
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 尝试连接到设备ID 1，使用正确的地址类型
        bt_mgr_err_t connect_result = bluetooth_manager_connect_device_with_type(
            result->mac_addr, 1, result->addr_type);
        if (connect_result == BT_MGR_OK) {
            ESP_LOGI(TAG, "[OK] Connection request sent successfully ^_^");
        } else {
            ESP_LOGW(TAG, "[ERR] Failed to send connection request: %d (x_x)", connect_result);
            ESP_LOGI(TAG, "[RETRY] Will retry after %lu seconds...", CONNECTION_RETRY_INTERVAL_MS/1000);
        }
    }
}

/* 设备状态变化回调函数 */
static void on_device_state_changed(uint8_t device_id, led_device_state_t old_state, led_device_state_t new_state) {
    const char* state_names[] = {"DISCONNECTED", "CONNECTING", "CONNECTED", "ERROR"};
    
    ESP_LOGI(TAG, "[STATE] Device ID %d: %s -> %s (@_@)", 
             device_id, state_names[old_state], state_names[new_state]);
    
    switch (new_state) {
        case LED_DEVICE_STATE_CONNECTING:
            ESP_LOGI(TAG, "[CONN] Device ID %d: Connection in progress... (~_~)", device_id);
            break;
            
        case LED_DEVICE_STATE_CONNECTED:
            ESP_LOGI(TAG, "[OK] Device ID %d: Successfully connected! \\(^o^)/", device_id);
            ESP_LOGI(TAG, "[READY] Ready to send commands to this device (^_^)");
            
            // 发送测试命令 - 设置为绿色3秒
            led_command_t test_cmd = {
                .type = CMD_LED_COLOR_WITH_TIME,  // 使用system_config.h中定义的命令类型
                .led_id = device_id,
                .param1 = LED_COLOR_GREEN, // 绿色
                .param2 = 3000, // 3秒 (低16位)
                .param3 = 0     // 3秒 (高16位)
            };
            
            ESP_LOGI(TAG, "[GREEN] Sending test command: GREEN light for 3 seconds (^_^)");
            bt_mgr_err_t cmd_result = bluetooth_manager_send_command(device_id, &test_cmd);
            if (cmd_result == BT_MGR_OK) {
                ESP_LOGI(TAG, "[SEND] Test command sent successfully (>.<)");
            } else {
                ESP_LOGW(TAG, "[ERR] Failed to send test command: %d (x_x)", cmd_result);
            }
            break;
            
        case LED_DEVICE_STATE_DISCONNECTED:
            ESP_LOGI(TAG, "[DISC] Device ID %d: Disconnected (-_-)", device_id);
            break;
            
        case LED_DEVICE_STATE_ERROR:
            ESP_LOGW(TAG, "[WARN] Device ID %d: Connection error (!_!)", device_id);
            break;
    }
}

/* 设备响应回调函数 */
static void on_device_response(uint8_t device_id, const uint8_t *data, uint16_t length) {
    ESP_LOGI(TAG, "[RECV] Received response from device ID %d (<_<)", device_id);
    ESP_LOGI(TAG, "[DATA] Length: %d bytes", length);
    
    if (length > 0) {
        ESP_LOGI(TAG, "[RAW] Raw data:");
        ESP_LOG_BUFFER_HEX(TAG, data, length);
        
        // 简单的协议解析（假设第一个字节是状态码）
        if (length >= 1) {
            switch (data[0]) {
                case 0x00:
                    ESP_LOGI(TAG, "[OK] Device response: SUCCESS (^_^)");
                    break;
                case 0x01:
                    ESP_LOGI(TAG, "[ERR] Device response: ERROR (x_x)");
                    break;
                case 0xAA:
                    ESP_LOGI(TAG, "[ACK] Device response: ACK/PING (^o^)");
                    break;
                default:
                    ESP_LOGI(TAG, "[STATUS] Device response: Status 0x%02X (o_o)", data[0]);
                    break;
            }
        }
    }
}

/* 状态监控任务 */
static void status_monitor_task(void *param) {
    ESP_LOGI(TAG, "[INIT] Status monitor task started \\(^o^)/");
    ESP_LOGI(TAG, "[BLE] BLE scan will start in 5 seconds... (~_~)");
    
    // 等待5秒让系统完全稳定，然后开始第一次扫描
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    uint32_t scan_counter = 10; // 立即触发第一次扫描
    const uint32_t scan_interval_cycles = BLE_SCAN_INTERVAL_MS / 10000; // 扫描间隔周期数
    
    while (1) {
        /* 每10秒检查一次系统状态 */
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        /* 更新系统运行时间 */
        g_system_status.uptime_seconds += 10;
        g_system_status.free_heap = esp_get_free_heap_size();
        
        /* 安全地获取蓝牙状态 */
        int connected_count = 0;
        bt_mgr_state_t bt_state = BT_MGR_STATE_UNINITIALIZED;
        
        // 添加错误检查，防止蓝牙模块未初始化时的函数调用
        if (g_system_status.bt_connected) {
            connected_count = bluetooth_manager_get_connected_count();
            bt_state = bluetooth_manager_get_state();
        }
        
        ESP_LOGI(TAG, "System status - BT state: %d, Connected devices: %d, Free heap: %lu", 
                bt_state, connected_count, g_system_status.free_heap);
        
        /* 定期扫描设备，但不要太频繁 */
        scan_counter++;
        if (connected_count == 0 && 
            bt_state == BT_MGR_STATE_READY && 
            scan_counter >= scan_interval_cycles) {
            
            ESP_LOGI(TAG, "[SCAN] Starting BLE device scan for %d seconds... (>_<)", BLE_SCAN_DURATION_MS/1000);
            ESP_LOGI(TAG, "[SEARCH] Looking for Smart Tags and other BLE devices...");
            
            bt_mgr_err_t scan_result = bluetooth_manager_start_scan(BLE_SCAN_DURATION_MS, on_device_found);
            if (scan_result == BT_MGR_OK) {
                ESP_LOGI(TAG, "[OK] BLE scan started successfully (^_^)");
            } else {
                ESP_LOGW(TAG, "[ERR] Failed to start BLE scan: %d (x_x)", scan_result);
            }
            scan_counter = 0; // 重置计数器
        } else if (scan_counter < scan_interval_cycles) {
            // 显示倒计时（仅在没有连接设备时）
            if (connected_count == 0 && bt_state == BT_MGR_STATE_READY) {
                uint32_t seconds_to_scan = (scan_interval_cycles - scan_counter) * 10;
                ESP_LOGI(TAG, "[TIME] Next BLE scan in %lu seconds... (T_T)", seconds_to_scan);
            }
        }
        
        /* 发送状态到MQTT（如果连接） */
        if (g_system_status.mqtt_connected && (g_system_status.uptime_seconds % 60 == 0)) {
            mqtt_send_status();
}
    }
}

void app_main(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "ESP32 Station starting up...");
    
    /* NVS Flash 初始化 */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS Flash initialized");
    
    /* 系统配置初始化 */
    system_config_init();
    ESP_LOGI(TAG, "System config initialized");
    
    /* 创建LED命令队列 */
    QueueHandle_t led_cmd_queue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(led_command_t));
    if (led_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create LED command queue");
        return;
    }
    ESP_LOGI(TAG, "LED command queue created");
    
    /* LED控制器初始化 */
    ret = led_controller_init(led_cmd_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED controller: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "LED controller initialized");
    
    /* 蓝牙管理器初始化 */
    bt_mgr_err_t bt_ret = bluetooth_manager_init();
    if (bt_ret != BT_MGR_OK) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth manager: %d", bt_ret);
        return;
    }
    ESP_LOGI(TAG, "Bluetooth manager initialized");
    
    /* 启动蓝牙管理器 */
    bt_ret = bluetooth_manager_start();
    if (bt_ret != BT_MGR_OK) {
        ESP_LOGE(TAG, "Failed to start Bluetooth manager: %d", bt_ret);
        return;
    }
    g_system_status.bt_connected = true;  // 标记蓝牙功能已启用
    ESP_LOGI(TAG, "Bluetooth manager started");
    
    /* 设置蓝牙回调函数 */
    bluetooth_manager_set_state_callback(on_device_state_changed);
    bluetooth_manager_set_response_callback(on_device_response);
    ESP_LOGI(TAG, "Bluetooth callbacks configured");
    
    /* MQTT管理器初始化 */
    ret = mqtt_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT manager: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "MQTT manager initialized");
    
    /* OTA管理器初始化 */
    ret = ota_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize OTA manager: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "OTA manager initialized");
    
    /* 创建状态监控任务 */
    xTaskCreate(status_monitor_task, "status_monitor", STATUS_MONITOR_STACK_SIZE, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "ESP32 Station initialization completed");
    ESP_LOGI(TAG, "Device Name: ESP32_Station_Central");
    ESP_LOGI(TAG, "Station ready - will scan for Smart Tags automatically");
}