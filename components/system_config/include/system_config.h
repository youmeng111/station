#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// 系统配置
#define STATION_TAG "STATION"
#define MAX_LED_STRIPS 10
#define COMMAND_QUEUE_SIZE 20

// WiFi配置
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define WIFI_MAXIMUM_RETRY 5

// MQTT配置
#define MQTT_BROKER_URL "mqtt://your-mqtt-broker.com"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "esp32_station"
#define MQTT_USERNAME "your_username"
#define MQTT_PASSWORD "your_password"

// MQTT主题
#define MQTT_TOPIC_COMMAND "station/command"
#define MQTT_TOPIC_STATUS "station/status"
#define MQTT_TOPIC_RESPONSE "station/response"

// 蓝牙配置 - 已关闭
// #define BT_DEVICE_NAME "ESP32_Station_BLE"    // 蓝牙功能已关闭
// #define BLE_DEVICE_NAME "ESP32_Station_BLE"   // 蓝牙功能已关闭
// #define SVC_INST_ID 0                         // 蓝牙功能已关闭
// #define SPP_SERVER_NAME "SPP_SERVER"          // 蓝牙功能已关闭

// OTA配置
#define OTA_URL_SIZE 256
#define OTA_RECEIVE_TIMEOUT 5000

// LED控制配置
#define LED_STRIP_COUNT 5  // 默认管理5个灯条

// 命令类型定义
typedef enum {
    CMD_LED_ON = 1,
    CMD_LED_OFF = 2,
    CMD_LED_TOGGLE = 3,
    CMD_LED_BRIGHTNESS = 4,
    CMD_LED_COLOR = 5,
    CMD_SYSTEM_STATUS = 6,
    CMD_OTA_UPDATE = 7,
    CMD_REBOOT = 8
} command_type_t;

// 命令结构体
typedef struct {
    command_type_t type;
    uint8_t led_id;      // 灯条ID (0-255)
    uint8_t param1;      // 参数1 (亮度/颜色R)
    uint8_t param2;      // 参数2 (颜色G)
    uint8_t param3;      // 参数3 (颜色B)
    char data[64];       // 扩展数据
} led_command_t;

// 系统状态
typedef struct {
    bool wifi_connected;
    bool mqtt_connected;
    bool bt_connected;      // 保留字段但蓝牙功能已关闭，始终为false
    uint32_t uptime_seconds;
    uint32_t free_heap;
    uint8_t led_status[MAX_LED_STRIPS];
} system_status_t;

// 全局变量声明
extern system_status_t g_system_status;

// 系统初始化函数
void system_config_init(void);

#endif // SYSTEM_CONFIG_H 