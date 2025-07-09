#include "system_config.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdio.h>

// 全局变量定义
system_status_t g_system_status = {0};

// MQTT主题字符串
char MQTT_TOPIC_COMMAND[64];
char MQTT_TOPIC_STATUS[64];
char MQTT_TOPIC_EXCEPTION[64];

void system_config_init(void)
{
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 命令队列将在main.c中创建

    // 初始化系统状态
    g_system_status.wifi_connected = false;
    g_system_status.mqtt_connected = false;
    g_system_status.bt_connected = false;
    g_system_status.uptime_seconds = 0;
    g_system_status.free_heap = 0;
    
    // 初始化LED状态数组
    for (int i = 0; i < MAX_LED_STRIPS; i++) {
        g_system_status.led_status[i] = 0;
    }
    
    // 构建MQTT主题字符串
    snprintf(MQTT_TOPIC_COMMAND, sizeof(MQTT_TOPIC_COMMAND), 
             MQTT_TOPIC_COMMAND_FORMAT, BASE_STATION_ID);
    snprintf(MQTT_TOPIC_STATUS, sizeof(MQTT_TOPIC_STATUS), 
             MQTT_TOPIC_STATUS_FORMAT, BASE_STATION_ID);
    snprintf(MQTT_TOPIC_EXCEPTION, sizeof(MQTT_TOPIC_EXCEPTION), 
             MQTT_TOPIC_EXCEPTION_FORMAT, BASE_STATION_ID);
    
    ESP_LOGI(STATION_TAG, "System configuration initialized");
} 
