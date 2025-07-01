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
#include "nvs_flash.h"

/* 组件头文件 */
#include "system_config.h"
#include "led_controller.h"
#include "bluetooth_manager.h"
#include "mqtt_manager.h"
#include "ota_manager.h"

static const char *TAG = "MAIN";

/* 状态监控任务 */
static void status_monitor_task(void *param) {
    ESP_LOGI(TAG, "Status monitor task started");
    
    while (1) {
        /* 每5秒检查一次系统状态 */
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        /* 如果BLE已连接，发送状态通知 */
        if (bluetooth_manager_is_connected()) {
            bluetooth_manager_notify_led_status();
        }
        
        ESP_LOGI(TAG, "System running, BT state: %d", bluetooth_manager_get_state());
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
    ESP_LOGI(TAG, "Bluetooth manager started");
    
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
    xTaskCreate(status_monitor_task, "status_monitor", 2048, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "ESP32 Station initialization completed");
    ESP_LOGI(TAG, "Device Name: ESP32_Station_BLE");
    ESP_LOGI(TAG, "Waiting for BLE connections...");
}