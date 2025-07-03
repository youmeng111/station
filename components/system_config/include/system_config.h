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

// WiFi配置 - 请修改为您的实际WiFi信息
#define WIFI_SSID "您的WiFi名称"              // 修改为您的WiFi名称
#define WIFI_PASSWORD "您的WiFi密码"          // 修改为您的WiFi密码
#define WIFI_MAXIMUM_RETRY 5

// MQTT配置 - 可以使用免费的公共MQTT服务器进行测试
#define MQTT_BROKER_URL "mqtt://broker.emqx.io"  // 免费公共MQTT服务器
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "esp32_station_test"      // 添加唯一后缀避免冲突
#define MQTT_USERNAME ""                         // 公共服务器通常不需要用户名
#define MQTT_PASSWORD ""                         // 公共服务器通常不需要密码

// 基站配置
#define BASE_STATION_ID "0001"  // 基站唯一标识符

// MQTT主题 - 符合平台协议规范
#define MQTT_TOPIC_COMMAND_FORMAT "platform/to/base_station/%s/command"
#define MQTT_TOPIC_STATUS_FORMAT "base_station/to/platform/%s/status"
#define MQTT_TOPIC_EXCEPTION_FORMAT "base_station/to/platform/%s/exception"

// 动态主题字符串
extern char MQTT_TOPIC_COMMAND[64];
extern char MQTT_TOPIC_STATUS[64];
extern char MQTT_TOPIC_EXCEPTION[64];

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

// BLE扫描配置 - 测试优化
#define BLE_SCAN_DURATION_MS 10000            // BLE扫描持续时间(毫秒) - 每次扫描10秒
#define BLE_SCAN_INTERVAL_MS 15000            // BLE扫描间隔(毫秒) - 每15秒扫描一次（测试用）

// 任务堆栈大小配置
#define STATUS_MONITOR_STACK_SIZE 4096        // 状态监控任务堆栈大小
#define LED_CONTROLLER_STACK_SIZE 3072        // LED控制器任务堆栈大小

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