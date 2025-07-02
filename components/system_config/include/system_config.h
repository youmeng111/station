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

// 蓝牙配置 - BLE功能已启用
#define BT_DEVICE_NAME "ESP32_Station"        // BLE设备名称
#define BLE_DEVICE_NAME "ESP32_Station"       // BLE广播名称
#define SVC_INST_ID 0                         // 服务实例ID
#define BLE_ADV_INTERVAL_MIN 500              // BLE广播最小间隔(ms)
#define BLE_ADV_INTERVAL_MAX 510              // BLE广播最大间隔(ms)

// BLE LED控制协议配置
#define BLE_MAX_PROTOCOL_DATA_LENGTH 18       // 协议数据最大长度
#define BLE_MAX_PROTOCOL_FRAME_LENGTH 25      // 协议帧最大长度
#define BLE_PROTOCOL_FRAME_HEADER_1 0xAA      // 协议帧头1
#define BLE_PROTOCOL_FRAME_HEADER_2 0x55      // 协议帧头2
#define BLE_PROTOCOL_FRAME_TAIL_1   0x55      // 协议帧尾1
#define BLE_PROTOCOL_FRAME_TAIL_2   0xAA      // 协议帧尾2

// OTA配置
#define OTA_URL_SIZE 256
#define OTA_RECEIVE_TIMEOUT 5000

// LED控制配置
#define LED_STRIP_COUNT 10  // 管理10个LED灯条，与LED控制器初始化保持一致

// 命令类型定义
typedef enum {
    CMD_LED_ON = 1,
    CMD_LED_OFF = 2,
    CMD_LED_TOGGLE = 3,
    CMD_LED_COLOR = 4,
    CMD_LED_COLOR_WITH_TIME = 5,
    CMD_SYSTEM_STATUS = 6,
    CMD_OTA_UPDATE = 7,
    CMD_REBOOT = 8
} command_type_t;

// 命令结构体
typedef struct {
    command_type_t type;
    uint8_t led_id;          // 灯条ID (0-255)
    uint8_t param1;          // 参数1 (颜色枚举值)
    uint32_t param2;         // 参数2 (时长ms，低16位)
    uint32_t param3;         // 参数3 (时长ms，高16位)
    char data[64];           // 扩展数据
} led_command_t;

// 系统状态
typedef struct {
    bool wifi_connected;
    bool mqtt_connected;
    bool bt_connected;      // BLE连接状态
    uint32_t uptime_seconds;
    uint32_t free_heap;
    uint8_t led_status[MAX_LED_STRIPS];
} system_status_t;

// 全局变量声明
extern system_status_t g_system_status;

// 系统初始化函数
void system_config_init(void);

#endif // SYSTEM_CONFIG_H 