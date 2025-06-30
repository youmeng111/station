/*
 * ESP32智能基站主程序
 * 基于ESP-IDF官方NimBLE模板重构
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

/* 项目头文件 */
#include "system_config.h"
#include "led_controller.h"
#include "mqtt_manager.h"
#include "ota_manager.h"

static const char *TAG = "MAIN";

/* 全局变量 */
QueueHandle_t command_queue = NULL;

/**
 * @brief 系统状态任务
 */
static void system_status_task(void *param)
{
    ESP_LOGI(TAG, "System status task started");
    
    while (1) {
        // 更新系统状态
        g_system_status.uptime_seconds = xTaskGetTickCount() / configTICK_RATE_HZ;
        g_system_status.free_heap = esp_get_free_heap_size();
        g_system_status.bt_connected = false;  // 蓝牙功能已禁用
        
        // 每10秒打印一次状态
        ESP_LOGI(TAG, "System Status - Uptime: %ld, Free Heap: %ld, BT: %s, WiFi: %s, MQTT: %s",
                 g_system_status.uptime_seconds,
                 g_system_status.free_heap,
                 "Disabled",  // 蓝牙已禁用
                 g_system_status.wifi_connected ? "Connected" : "Disconnected",
                 g_system_status.mqtt_connected ? "Connected" : "Disconnected");
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/**
 * @brief WiFi和MQTT连接任务
 */
static void connectivity_task(void *param)
{
    ESP_LOGI(TAG, "Connectivity task started");
    
    // 初始化MQTT管理器（包含WiFi连接）
    if (mqtt_manager_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT manager");
    }
    
    // 初始化OTA管理器
    if (ota_manager_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize OTA manager");
    }
    
    while (1) {
        // 检查和维护连接
        if (!g_system_status.wifi_connected) {
            ESP_LOGI(TAG, "WiFi not connected, attempting reconnection...");
            // MQTT管理器会自动处理WiFi重连
        }
        
        if (!g_system_status.mqtt_connected && g_system_status.wifi_connected) {
            ESP_LOGI(TAG, "MQTT not connected, attempting reconnection...");
            mqtt_manager_reconnect();
        }
        
        vTaskDelay(pdMS_TO_TICKS(30000)); // 30秒检查一次
    }
}

/**
 * @brief 应用程序主函数
 */
void app_main(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "ESP32智能基站启动...");
    
    /* 初始化NVS存储 */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    /* 初始化系统配置 */
    system_config_init();
    
    /* 创建命令队列 */
    command_queue = xQueueCreate(10, sizeof(led_command_t));
    if (command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue");
        return;
    }
    
    /* 初始化LED控制器 */
    ret = led_controller_init(command_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED controller: %s", esp_err_to_name(ret));
        return;
    }
    
    /* 创建系统任务 */
    xTaskCreate(system_status_task, "sys_status", 4096, NULL, 5, NULL);
    xTaskCreate(connectivity_task, "connectivity", 8192, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "ESP32智能基站初始化完成");
    ESP_LOGI(TAG, "系统功能:");
    ESP_LOGI(TAG, "  - LED灯条控制");
    ESP_LOGI(TAG, "  - WiFi+MQTT远程控制");
    ESP_LOGI(TAG, "  - HTTP/HTTPS OTA升级");
    ESP_LOGI(TAG, "设备名称: ESP32-Smart-Station");
    ESP_LOGI(TAG, "注意: 蓝牙功能已禁用");
    
    /* 主循环 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}