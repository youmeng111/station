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

/* BLE广播配置 */
#define BLE_ADV_INTERVAL_MIN 100         // 广播最小间隔(ms) - 更频繁的广播
#define BLE_ADV_INTERVAL_MAX 150         // 广播最大间隔(ms)
#define BLE_ADV_REPEAT_COUNT 3           // 每个命令重复广播次数
#define BLE_ADV_REPEAT_INTERVAL 50       // 重复广播间隔(ms)

/* 广播数据包格式定义 */
#define BROADCAST_PACKET_HEADER_1 0xAA   // 数据包头1
#define BROADCAST_PACKET_HEADER_2 0x55   // 数据包头2
#define BROADCAST_PACKET_TAIL     0x99   // 数据包尾
#define MAX_BROADCAST_DATA_LENGTH 25     // BLE广播数据最大有效载荷(去除厂商标识后)

/* 广播命令类型定义 */
#define BROADCAST_CMD_SET_COLOR       0x01  // 设置颜色
#define BROADCAST_CMD_SET_BLINK       0x02  // 设置闪烁模式
#define BROADCAST_CMD_SET_BEEP        0x03  // 设置蜂鸣器
#define BROADCAST_CMD_GROUP_ACTIVATE  0x04  // 分组激活
#define BROADCAST_CMD_ALL_OFF         0x05  // 全部关闭
#define BROADCAST_CMD_BATTERY_QUERY   0x06  // 电池查询广播
#define BROADCAST_CMD_STATUS_SYNC     0x07  // 状态同步
#define BROADCAST_CMD_BATTERY_WARNING 0x08  // 电池低电预警
#define BROADCAST_CMD_INVENTORY_SCAN  0x09  // 盘点扫描
#define BROADCAST_CMD_HEALTH_CHECK    0x0A  // 健康检查
#define BROADCAST_CMD_FAULT_REPORT    0x0B  // 故障上报

/* 设备分组定义 - 快递驿站场景 */
#define GROUP_ALL           0xFF    // 所有设备
#define GROUP_SHELF_A       0x01    // 货架A区域
#define GROUP_SHELF_B       0x02    // 货架B区域
#define GROUP_SHELF_C       0x03    // 货架C区域
#define GROUP_SHELF_D       0x04    // 货架D区域
#define GROUP_COUNTER       0x10    // 取件台区域
#define GROUP_ENTRANCE      0x20    // 入口区域
#define GROUP_SPECIAL       0x30    // 特殊包裹区域
#define GROUP_MAINTENANCE   0xF0    // 维护模式组

/* 颜色定义 */
#define LED_COLOR_OFF       0x00
#define LED_COLOR_RED       0x01
#define LED_COLOR_GREEN     0x02
#define LED_COLOR_BLUE      0x03
#define LED_COLOR_YELLOW    0x04
#define LED_COLOR_PURPLE    0x05
#define LED_COLOR_CYAN      0x06
#define LED_COLOR_WHITE     0x07

/* 闪烁模式定义 */
#define BLINK_MODE_NONE     0x00    // 常亮
#define BLINK_MODE_SLOW     0x01    // 慢闪(1Hz)
#define BLINK_MODE_FAST     0x02    // 快闪(3Hz)
#define BLINK_MODE_URGENT   0x03    // 紧急闪烁(5Hz)

/* 蜂鸣器模式定义 */
#define BEEP_MODE_OFF       0x00    // 关闭
#define BEEP_MODE_SINGLE    0x01    // 单次哔声
#define BEEP_MODE_DOUBLE    0x02    // 双次哔声
#define BEEP_MODE_CONTINUOUS 0x03   // 连续哔声

/* 优先级定义 */
#define PRIORITY_LOW        0x00    // 低优先级
#define PRIORITY_NORMAL     0x01    // 普通优先级
#define PRIORITY_HIGH       0x02    // 高优先级
#define PRIORITY_EMERGENCY  0x03    // 紧急优先级

/* BLE广播数据包结构 (最大31字节) - 无校验版本 */
typedef struct __attribute__((packed)) {
    // BLE广播标准字段
    uint8_t length;                 // 数据长度
    uint8_t type;                   // 数据类型(0xFF = 厂商数据)
    uint16_t company_id;            // 厂商ID (0x02E5 = Espressif)
    
    // 自定义广播协议 (最大26字节)
    uint8_t header[2];              // 协议头 {0xAA, 0x55}
    uint8_t cmd_type;               // 命令类型
    uint8_t group_id;               // 目标分组ID
    uint16_t sequence;              // 序列号(防重复执行)
    uint8_t priority;               // 优先级
    
    // 命令参数 (17字节)
    union {
        struct {
            uint8_t color;          // LED颜色
            uint8_t blink_mode;     // 闪烁模式
            uint8_t beep_mode;      // 蜂鸣器模式
            uint32_t duration_ms;   // 持续时间(毫秒)
            uint8_t reserved[10];   // 保留字节
        } led_control;
        
        struct {
            uint32_t tag_ids[4];    // 特定标签ID列表
            uint8_t reserved;       // 保留字节
        } targeted_control;
        
        struct {
            uint8_t battery_level;  // 电池电量百分比
            uint16_t voltage_mv;    // 电池电压(毫伏)
            uint8_t warning_type;   // 预警类型
            uint8_t reserved[13];   // 保留字节
        } battery_warning;
        
        struct {
            uint32_t scan_id;       // 盘点ID
            uint8_t scan_type;      // 盘点类型 (0=全部, 1=分组, 2=指定)
            uint32_t timeout_ms;    // 响应超时时间
            uint8_t reserved[8];    // 保留字节
        } inventory_scan;
        
        struct {
            uint8_t check_items;    // 检查项目位掩码
            uint32_t check_id;      // 检查ID
            uint8_t reserved[12];   // 保留字节
        } health_check;
        
        struct {
            uint32_t tag_id;        // 标签ID
            uint8_t fault_type;     // 故障类型
            uint8_t fault_code;     // 故障代码
            uint8_t battery_level;  // 当前电量
            uint16_t voltage_mv;    // 当前电压
            uint8_t led_status;     // LED状态
            uint8_t beep_status;    // 蜂鸣器状态
            uint8_t reserved[8];    // 保留字节
        } fault_report;
        
        uint8_t raw_data[17];       // 原始数据
    } params;
    
    uint8_t tail;                   // 协议尾 0x99
} broadcast_packet_t;

/* 广播管理器状态 */
typedef enum {
    BROADCAST_STATE_IDLE = 0,
    BROADCAST_STATE_ADVERTISING,
    BROADCAST_STATE_REPEATING,
    BROADCAST_STATE_ERROR
} broadcast_state_t;

/* 设备分组信息 */
typedef struct {
    uint8_t group_id;
    char group_name[16];
    uint32_t estimated_device_count;
    bool is_active;
    uint32_t last_command_time;
} device_group_t;

/* 广播统计信息 */
typedef struct {
    uint32_t total_broadcasts;
    uint32_t successful_broadcasts;
    uint32_t failed_broadcasts;
    uint32_t last_broadcast_time;
    uint16_t current_sequence;
} broadcast_stats_t;

/* LED命令结构 - 使用system_config.h中的定义 */
typedef led_command_t ble_led_cmd_t;

/* 广播控制接口函数声明 */
uint16_t pack_broadcast_packet(const broadcast_packet_t *packet, uint8_t *buffer, uint16_t buffer_size);
bool unpack_broadcast_packet(const uint8_t *buffer, uint16_t length, broadcast_packet_t *packet);

/* 盘点信息结构 */
typedef struct {
    uint32_t tag_id;                // 标签ID
    uint8_t battery_level;          // 电池电量
    uint16_t voltage_mv;            // 电池电压
    uint8_t led_status;             // LED状态
    uint8_t beep_status;            // 蜂鸣器状态
    uint8_t signal_strength;        // 信号强度
    uint32_t last_seen_time;        // 最后检测时间
    uint8_t group_id;               // 所属分组
    uint8_t fault_flags;            // 故障标志位
} inventory_info_t;

/* 健康检查项目定义 */
#define HEALTH_CHECK_BATTERY    0x01    // 检查电池
#define HEALTH_CHECK_LED        0x02    // 检查LED
#define HEALTH_CHECK_BEEP       0x04    // 检查蜂鸣器
#define HEALTH_CHECK_SIGNAL     0x08    // 检查信号
#define HEALTH_CHECK_ALL        0x0F    // 检查所有项目

/* 故障类型定义 */
#define FAULT_TYPE_BATTERY_LOW      0x01    // 电池低电
#define FAULT_TYPE_BATTERY_CRITICAL 0x02    // 电池严重不足
#define FAULT_TYPE_LED_FAILURE      0x03    // LED故障
#define FAULT_TYPE_BEEP_FAILURE     0x04    // 蜂鸣器故障
#define FAULT_TYPE_SIGNAL_WEAK      0x05    // 信号弱
#define FAULT_TYPE_COMMUNICATION    0x06    // 通信故障

#endif // BT_COMMON_H 