#ifndef BT_COMMON_H
#define BT_COMMON_H

/* Includes */
/* STD APIs */
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ESP APIs */
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* FreeRTOS APIs */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* NimBLE stack APIs */
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

/* LED Controller */
#include "led_controller.h"

/* Defines */
#define BT_TAG "ESP32_Station_BLE"
#define DEVICE_NAME "ESP32_Station"

/* BLE地址格式宏 */
#define ADDR_FORMAT "%02x:%02x:%02x:%02x:%02x:%02x"
#define ADDR_FORMAT_ARGS(addr) \
    addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]

/* BLE广播间隔(ms) */
#define BLE_ADV_INTERVAL_MIN 500
#define BLE_ADV_INTERVAL_MAX 510

/* 灯条协议帧格式定义 */
#define PROTOCOL_FRAME_HEADER_1 0xAA
#define PROTOCOL_FRAME_HEADER_2 0x55
#define PROTOCOL_FRAME_TAIL_1   0x55
#define PROTOCOL_FRAME_TAIL_2   0xAA
#define MAX_PROTOCOL_DATA_LENGTH 18
#define MAX_PROTOCOL_FRAME_LENGTH (MAX_PROTOCOL_DATA_LENGTH + 7) // 帧头(2) + 长度(1) + 数据(18) + 校验(2) + 帧尾(2)

/* 灯条协议命令类型定义 */
#define CMD_SET_LIGHT_DURATION  0x01  // 设置亮灯时长
#define CMD_SET_LIGHT_COLOR     0x02  // 设置亮灯颜色
#define CMD_SET_BLINK_MODE      0x03  // 设置闪烁模式
#define CMD_SET_BEEP_STATE      0x04  // 设置蜂鸣器状态
#define CMD_SET_CONTROL_MODE    0x05  // 设置控制模式
#define CMD_GET_BATTERY_STATUS  0x06  // 获取/上报电池状态
#define CMD_BATTERY_LOW_ALERT   0x07  // 电池电量低警报

/* 灯条颜色定义 */
#define LED_COLOR_RED     0x00
#define LED_COLOR_GREEN   0x01
#define LED_COLOR_BLUE    0x02
#define LED_COLOR_YELLOW  0x03
#define LED_COLOR_PURPLE  0x04
#define LED_COLOR_CYAN    0x05
#define LED_COLOR_WHITE   0x06
#define LED_COLOR_OFF     0x07

/* 闪烁模式定义 */
#define BLINK_MODE_NONE   0x00  // 不闪烁(常亮)
#define BLINK_MODE_SLOW   0x01  // 慢闪(1Hz)
#define BLINK_MODE_FAST   0x02  // 快闪(2Hz)

/* 蜂鸣器状态定义 */
#define BEEP_STATE_OFF    0x00  // 关闭
#define BEEP_STATE_ON     0x01  // 开启

/* 控制模式定义 */
#define CONTROL_MODE_SINGLE 0x00  // 单控模式
#define CONTROL_MODE_ALL    0x01  // 全控模式

/* 电池电量等级定义 */
#define BATTERY_LEVEL_CRITICAL  0x00  // 严重不足
#define BATTERY_LEVEL_LOW       0x01  // 低
#define BATTERY_LEVEL_MEDIUM    0x02  // 中等
#define BATTERY_LEVEL_HIGH      0x03  // 高
#define BATTERY_LEVEL_FULL      0x04  // 满电

/* 设备ID定义 */
#define DEVICE_ID_BROADCAST 0xFFFF  // 广播地址

/* 协议错误代码 */
#define PROTOCOL_OK                 0x00
#define PROTOCOL_ERR_FRAME_HEAD     0x01
#define PROTOCOL_ERR_FRAME_TAIL     0x02
#define PROTOCOL_ERR_LENGTH         0x03
#define PROTOCOL_ERR_CHECKSUM       0x04
#define PROTOCOL_ERR_BUFFER_FULL    0x05

/* LED命令结构 - 使用system_config.h中的定义 */
typedef led_command_t ble_led_cmd_t;

/* 灯条协议帧结构 */
typedef struct __attribute__((packed)) {
    uint8_t header[2];          // 帧头 AA 55
    uint8_t length;             // 数据长度
    uint8_t data[MAX_PROTOCOL_DATA_LENGTH]; // 数据部分
    uint16_t checksum;          // 16位和校验(大端格式)
    uint8_t tail[2];            // 帧尾 55 AA
} protocol_frame_t;

/* 基站发送到灯条的命令数据结构 */
typedef struct __attribute__((packed)) {
    uint8_t cmd_type;           // 命令类型
    uint8_t data_length;        // 数据长度
    uint16_t device_id;         // 设备ID(大端格式)
    uint8_t cmd_data[14];       // 命令数据(最大14字节)
} station_to_strip_cmd_t;

/* 灯条发送到基站的数据结构 */
typedef struct __attribute__((packed)) {
    uint8_t cmd_type;           // 命令类型
    uint16_t device_id;         // 设备ID(大端格式)
    uint8_t data[15];           // 数据内容(最大15字节)
} strip_to_station_data_t;

/* 协议帧处理函数声明 */
uint16_t protocol_frame_pack(uint8_t *frame_buf, const uint8_t *data, uint16_t data_len);
uint16_t protocol_frame_unpack(const uint8_t *frame_buf, uint16_t frame_len, uint8_t *data_buf, uint16_t data_buf_size);
uint16_t checksum16_calculate(const uint8_t *data, uint16_t length);
uint16_t create_response_frame(uint8_t cmd_type, uint16_t device_id, const uint8_t *data, uint8_t data_len, uint8_t *frame_buf, uint16_t frame_buf_size);

#endif // BT_COMMON_H 