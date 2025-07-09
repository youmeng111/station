/*
 * LED Color Test Demo
 * Function: Scan and connect to smart tag devices, then cycle through LED colors
 * 
 * Color cycle sequence: Red -> Green -> Blue -> Yellow -> Purple -> Cyan -> White -> Off -> Repeat
 * Each color lasts for 3 seconds
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"

/* 组件头文件 */
#include "system_config.h"
#include "led_controller.h"
#include "bluetooth_manager.h"

static const char *TAG = "LED_COLOR_DEMO";

/* LED颜色循环数组 */
static const led_color_t color_sequence[] = {
    LED_COLOR_RED,      // 红色
    LED_COLOR_GREEN,    // 绿色
    LED_COLOR_BLUE,     // 蓝色
    LED_COLOR_YELLOW,   // 黄色
    LED_COLOR_PURPLE,   // 紫色
    LED_COLOR_CYAN,     // 青色
    LED_COLOR_WHITE,    // 白色
    LED_COLOR_OFF       // 关闭
};

/* 颜色名称数组（用于日志显示） */
static const char* color_names[] = {
    "OFF", "RED", "GREEN", "BLUE", "YELLOW", "PURPLE", "CYAN", "WHITE"
};

/* 全局变量 */
static uint8_t g_current_color_index = 0;
static const uint8_t g_color_count = sizeof(color_sequence) / sizeof(color_sequence[0]);
static bool g_demo_running = false;
static uint8_t g_connected_device_id = 0;
static bool g_services_discovered = false;
static uint32_t g_command_sent_count = 0;
static uint32_t g_response_received_count = 0;

/* LED颜色循环任务 */
static void led_color_cycle_task(void *param) {
    ESP_LOGI(TAG, "[TASK] LED color cycle task started");
    
    while (1) {
        /* 检查是否有连接的设备 */
        if (g_demo_running && g_connected_device_id > 0) {
            /* Check if services are discovered - 简化检查 */
            if (!g_services_discovered) {
                ESP_LOGW(TAG, "[CYCLE] Services not discovered, waiting...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            
            /* Get current color */
            led_color_t current_color = color_sequence[g_current_color_index];
            const char* color_name = color_names[current_color];
            
            ESP_LOGI(TAG, "[CYCLE] [%d/%d] Sending: %s", 
                    g_current_color_index + 1, g_color_count, color_name);
            
            /* 构造LED颜色命令 */
            led_command_t color_cmd = {
                .type = CMD_LED_COLOR_WITH_TIME,  // 使用system_config.h中定义的命令类型
                .led_id = g_connected_device_id,
                .param1 = current_color,
                .param2 = 3000,  // 3秒持续时间（低16位）
                .param3 = 0      // 高16位
            };
            
            /* Send command */
            g_command_sent_count++;
            bt_mgr_err_t result = bluetooth_manager_send_command(g_connected_device_id, &color_cmd);
            if (result == BT_MGR_OK) {
                ESP_LOGI(TAG, "[CYCLE] Command sent: %s", color_name);
            } else {
                ESP_LOGW(TAG, "[CYCLE] Command failed: %s (error: %d)", color_name, result);
            }
            
            /* 移到下一个颜色 */
            g_current_color_index = (g_current_color_index + 1) % g_color_count;
            
            /* 等待3.5秒再发送下一个颜色（给设备留出处理时间） */
            vTaskDelay(pdMS_TO_TICKS(3500));
        } else {
            /* Wait for connection */
            ESP_LOGI(TAG, "[CYCLE] Waiting for device connection...");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

/* 设备发现回调函数 */
static void demo_on_device_found(const scan_result_t *result) {
    ESP_LOGI(TAG, "[SCAN] Found: %s", result->name);
    
    /* 如果发现智能标签，尝试连接 */
    if (result->is_led_device && !g_demo_running) {
        ESP_LOGI(TAG, "[CONNECT] Connecting to %s...", result->name);
        
        /* 连接到设备，使用设备ID为1 */
        bt_mgr_err_t connect_result = bluetooth_manager_connect_device_with_type(
            result->mac_addr, 1, result->addr_type);
        
        if (connect_result != BT_MGR_OK) {
            ESP_LOGW(TAG, "[CONNECT] Failed: %d", connect_result);
        }
    }
}

/* 设备状态变化回调函数 */
static void demo_on_device_state_changed(uint8_t device_id, led_device_state_t old_state, led_device_state_t new_state) {
    ESP_LOGI(TAG, "[STATE] Device %d: %d -> %d", device_id, old_state, new_state);
    
    switch (new_state) {
        case LED_DEVICE_STATE_CONNECTING:
            ESP_LOGI(TAG, "[STATE] Device %d connecting...", device_id);
            break;
            
        case LED_DEVICE_STATE_CONNECTED:
            ESP_LOGI(TAG, "[STATE] Device %d connected!", device_id);
            g_connected_device_id = device_id;
            g_demo_running = true;
            g_services_discovered = true;  // 直接设置为true
            
            // 发送初始化命令 - 关闭LED
            led_command_t init_cmd = {
                .type = CMD_LED_OFF,
                .led_id = device_id,
                .param1 = 0,
                .param2 = 0,
                .param3 = 0
            };
            
            bt_mgr_err_t init_result = bluetooth_manager_send_command(device_id, &init_cmd);
            if (init_result == BT_MGR_OK) {
                ESP_LOGI(TAG, "[STATE] Init command sent");
                g_command_sent_count++;
            } else {
                ESP_LOGW(TAG, "[STATE] Init command failed: %d", init_result);
            }
            break;
            
        case LED_DEVICE_STATE_DISCONNECTED:
            ESP_LOGI(TAG, "[STATE] Device %d disconnected", device_id);
            if (g_connected_device_id == device_id) {
                g_demo_running = false;
                g_connected_device_id = 0;
                g_services_discovered = false;
            }
            break;
            
        case LED_DEVICE_STATE_ERROR:
            ESP_LOGW(TAG, "[STATE] Device %d error", device_id);
            if (g_connected_device_id == device_id) {
                g_demo_running = false;
                g_connected_device_id = 0;
                g_services_discovered = false;
            }
            break;
    }
}

/* 设备响应回调函数 */
static void demo_on_device_response(uint8_t device_id, const uint8_t *data, uint16_t length) {
    g_response_received_count++;
    ESP_LOGI(TAG, "[RESPONSE] Device %d: %d bytes", device_id, length);
    
    if (length > 0 && data) {
        ESP_LOGI(TAG, "[RESPONSE] Status: 0x%02X", data[0]);
    }
}

/* Scan task */
static void demo_scan_task(void *param) {
    ESP_LOGI(TAG, "[TASK] Scan task started");
    
    /* Wait for system initialization to complete */
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    while (1) {
        /* If no device connected, start scanning */
        if (!g_demo_running && g_connected_device_id == 0) {
            bt_mgr_state_t bt_state = bluetooth_manager_get_state();
            
            if (bt_state == BT_MGR_STATE_READY) {
                ESP_LOGI(TAG, "[SCAN] Starting scan...");
                
                bt_mgr_err_t scan_result = bluetooth_manager_start_scan(10000, demo_on_device_found);
                if (scan_result != BT_MGR_OK) {
                    ESP_LOGW(TAG, "[SCAN] Failed: %d", scan_result);
                }
            } else {
                ESP_LOGI(TAG, "[SCAN] BT not ready: %d", bt_state);
            }
        }
        
        /* Wait 30 seconds before next scan */
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

/* Status display task */
static void demo_status_task(void *param) {
    ESP_LOGI(TAG, "[TASK] Status display task started");
    
    while (1) {
        /* Display status every 15 seconds */
        vTaskDelay(pdMS_TO_TICKS(15000));
        
        // 极简状态打印，避免栈溢出
        ESP_LOGI(TAG, "[STATUS] Heap:%lu Dev:%d Run:%s Cmd:%lu Rsp:%lu", 
                 esp_get_free_heap_size(), g_connected_device_id, 
                 g_demo_running ? "Y" : "N", g_command_sent_count, g_response_received_count);
    }
}

/* Main function */
void app_main(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "[MAIN] LED Color Test Demo starting");
    
    /* NVS Flash initialization */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    /* System configuration initialization */
    system_config_init();
    
    /* Create LED command queue */
    QueueHandle_t led_cmd_queue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(led_command_t));
    if (led_cmd_queue == NULL) {
        ESP_LOGE(TAG, "[MAIN] LED command queue creation failed");
        return;
    }
    
    /* LED controller initialization */
    ret = led_controller_init(led_cmd_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[MAIN] LED controller initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    /* Bluetooth manager initialization */
    bt_mgr_err_t bt_ret = bluetooth_manager_init();
    if (bt_ret != BT_MGR_OK) {
        ESP_LOGE(TAG, "[MAIN] Bluetooth manager initialization failed: %d", bt_ret);
        return;
    }
    
    /* Start Bluetooth manager */
    bt_ret = bluetooth_manager_start();
    if (bt_ret != BT_MGR_OK) {
        ESP_LOGE(TAG, "[MAIN] Bluetooth manager start failed: %d", bt_ret);
        return;
    }
    
    /* Set Bluetooth callback functions */
    bluetooth_manager_set_state_callback(demo_on_device_state_changed);
    bluetooth_manager_set_response_callback(demo_on_device_response);
    
    /* Create demo tasks */
    xTaskCreate(demo_scan_task, "demo_scan", 4096, NULL, 5, NULL);
    xTaskCreate(led_color_cycle_task, "led_cycle", 4096, NULL, 4, NULL);
    xTaskCreate(demo_status_task, "demo_status", 4096, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "[MAIN] Demo started - scanning for devices...");
} 