#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include "common.h"
#include <stdint.h>
#include <stdbool.h>

/* 广播管理器配置 */
#define MAX_DEVICE_GROUPS           16      // 最大分组数量
#define BROADCAST_TIMEOUT_MS        1000    // 广播超时时间
#define SEQUENCE_HISTORY_SIZE       32      // 序列号历史记录大小

/* 蓝牙管理器错误代码 */
typedef enum {
    BT_MGR_OK = 0,
    BT_MGR_ERR_INIT_FAILED = -1,
    BT_MGR_ERR_NOT_INITIALIZED = -2,
    BT_MGR_ERR_INVALID_PARAM = -3,
    BT_MGR_ERR_BROADCAST_FAILED = -4,
    BT_MGR_ERR_GROUP_NOT_FOUND = -5,
    BT_MGR_ERR_SEQUENCE_OVERFLOW = -6,
    BT_MGR_ERR_PACKET_TOO_LARGE = -7
} bt_mgr_err_t;

/* 蓝牙管理器状态 */
typedef enum {
    BT_MGR_STATE_UNINITIALIZED = 0,
    BT_MGR_STATE_INITIALIZED,
    BT_MGR_STATE_READY,
    BT_MGR_STATE_BROADCASTING,
    BT_MGR_STATE_ERROR
} bt_mgr_state_t;

/* 广播命令结构 */
typedef struct {
    uint8_t cmd_type;               // 命令类型 (BROADCAST_CMD_xxx)
    uint8_t group_id;               // 目标分组 (GROUP_xxx)
    uint8_t priority;               // 优先级 (PRIORITY_xxx)
    
    // 命令参数
    union {
        struct {
            uint8_t color;          // LED颜色
            uint8_t blink_mode;     // 闪烁模式
            uint8_t beep_mode;      // 蜂鸣器模式
            uint32_t duration_ms;   // 持续时间
        } led_params;
        
        struct {
            uint32_t tag_ids[4];    // 特定标签ID
            uint8_t count;          // 标签数量
        } target_params;
        
        struct {
            uint8_t battery_level;  // 电池电量百分比
            uint16_t voltage_mv;    // 电池电压(毫伏)
            uint8_t warning_type;   // 预警类型
        } battery_warning;
        
        struct {
            uint32_t scan_id;       // 盘点ID
            uint8_t scan_type;      // 盘点类型
            uint32_t timeout_ms;    // 响应超时时间
        } inventory_params;
        
        struct {
            uint8_t check_items;    // 检查项目位掩码
            uint32_t check_id;      // 检查ID
        } health_check_params;
        
        uint8_t raw_data[16];       // 原始数据
    } params;
} broadcast_command_t;

/* 广播响应回调函数类型 */
typedef void (*broadcast_sent_cb_t)(bt_mgr_err_t result, uint16_t sequence);

/* 设备反馈回调函数类型 (如果设备通过其他方式反馈) */
typedef void (*device_feedback_cb_t)(uint32_t tag_id, uint8_t response_type, const uint8_t *data, uint8_t length);

/* 盘点结果回调函数类型 */
typedef void (*inventory_result_cb_t)(uint32_t scan_id, const inventory_info_t *info);

/* 健康检查结果回调函数类型 */
typedef void (*health_check_result_cb_t)(uint32_t check_id, uint32_t tag_id, uint8_t check_result, uint8_t fault_flags);

/* 故障报告回调函数类型 */
typedef void (*fault_report_cb_t)(uint32_t tag_id, uint8_t fault_type, uint8_t fault_code, const uint8_t *fault_data);

/* =================== 基站蓝牙管理器API =================== */

/**
 * @brief 初始化广播蓝牙管理器
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_init(void);

/**
 * @brief 启动广播服务
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_start(void);

/**
 * @brief 停止广播服务
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_stop(void);

/**
 * @brief 获取管理器状态
 * @return 当前状态
 */
bt_mgr_state_t bluetooth_manager_get_state(void);

/* =================== 广播控制接口 =================== */

/**
 * @brief 发送广播命令
 * @param cmd 广播命令结构
 * @param sent_cb 发送完成回调 (可选)
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_broadcast_command(const broadcast_command_t *cmd, broadcast_sent_cb_t sent_cb);

/**
 * @brief 广播到所有设备 - 设置LED颜色
 * @param color LED颜色
 * @param blink_mode 闪烁模式
 * @param duration_ms 持续时间(0=永久)
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_broadcast_all_set_color(uint8_t color, uint8_t blink_mode, uint32_t duration_ms);

/**
 * @brief 广播到指定分组 - 设置LED颜色
 * @param group_id 分组ID
 * @param color LED颜色
 * @param blink_mode 闪烁模式
 * @param duration_ms 持续时间(0=永久)
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_broadcast_group_set_color(uint8_t group_id, uint8_t color, uint8_t blink_mode, uint32_t duration_ms);

/**
 * @brief 广播电池低电预警
 * @param battery_level 电池电量百分比
 * @param voltage_mv 电池电压(毫伏)
 * @param warning_type 预警类型
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_broadcast_battery_warning(uint8_t battery_level, uint16_t voltage_mv, uint8_t warning_type);

/**
 * @brief 发起盘点扫描
 * @param scan_id 盘点ID
 * @param scan_type 盘点类型 (0=全部, 1=分组, 2=指定)
 * @param group_id 分组ID (scan_type=1时有效)
 * @param timeout_ms 响应超时时间
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_start_inventory_scan(uint32_t scan_id, uint8_t scan_type, uint8_t group_id, uint32_t timeout_ms);

/**
 * @brief 发起健康检查
 * @param check_items 检查项目位掩码
 * @param check_id 检查ID
 * @param group_id 分组ID
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_start_health_check(uint8_t check_items, uint32_t check_id, uint8_t group_id);

/**
 * @brief 广播关闭所有设备
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_broadcast_all_off(void);

/**
 * @brief 广播蜂鸣器控制
 * @param group_id 分组ID (GROUP_ALL表示所有设备)
 * @param beep_mode 蜂鸣器模式
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_broadcast_beep(uint8_t group_id, uint8_t beep_mode);

/* =================== 分组管理接口 =================== */

/**
 * @brief 注册设备分组
 * @param group_id 分组ID
 * @param group_name 分组名称
 * @param estimated_count 预估设备数量
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_register_group(uint8_t group_id, const char *group_name, uint32_t estimated_count);

/**
 * @brief 获取分组信息
 * @param group_id 分组ID
 * @return 分组信息指针，失败返回NULL
 */
const device_group_t* bluetooth_manager_get_group(uint8_t group_id);

/**
 * @brief 获取所有分组列表
 * @param groups 分组数组缓冲区
 * @param max_count 最大分组数量
 * @return 实际分组数量
 */
int bluetooth_manager_get_all_groups(device_group_t *groups, int max_count);

/**
 * @brief 激活分组
 * @param group_id 分组ID
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_activate_group(uint8_t group_id);

/**
 * @brief 停用分组
 * @param group_id 分组ID
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_deactivate_group(uint8_t group_id);

/* =================== 高级控制接口 =================== */

/**
 * @brief 批量激活标签 (快递驿站取件场景)
 * @param package_ids 包裹位置ID数组
 * @param count 数量
 * @param color 激活颜色
 * @param enable_beep 是否启用蜂鸣器
 * @return 成功处理的数量
 */
int bluetooth_manager_activate_multiple_tags(const uint32_t *package_ids, int count, uint8_t color, bool enable_beep);

/**
 * @brief 快递驿站模式 (所有设备蓝色常亮)
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_express_station_mode(void);

/**
 * @brief 包裹取件提醒 (指定区域设备闪烁提醒)
 * @param group_id 区域分组ID
 * @param alert_color 提醒颜色
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_package_pickup_alert(uint8_t group_id, uint8_t alert_color);

/**
 * @brief 夜间模式广播 (所有设备低亮度)
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_night_mode(void);

/* =================== 配置和统计接口 =================== */

/**
 * @brief 设置广播间隔
 * @param interval_ms 广播间隔(毫秒)
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_set_broadcast_interval(uint32_t interval_ms);

/**
 * @brief 设置广播重复次数
 * @param repeat_count 重复次数
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_set_repeat_count(uint8_t repeat_count);

/**
 * @brief 获取广播统计信息
 * @return 统计信息指针
 */
const broadcast_stats_t* bluetooth_manager_get_stats(void);

/**
 * @brief 重置广播统计
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_reset_stats(void);

/**
 * @brief 设置广播完成回调
 * @param cb 回调函数
 */
void bluetooth_manager_set_broadcast_callback(broadcast_sent_cb_t cb);

/**
 * @brief 设置设备反馈回调 (如果设备有反馈机制)
 * @param cb 回调函数
 */
void bluetooth_manager_set_feedback_callback(device_feedback_cb_t cb);

/**
 * @brief 设置盘点结果回调
 * @param cb 回调函数
 */
void bluetooth_manager_set_inventory_callback(inventory_result_cb_t cb);

/**
 * @brief 设置健康检查结果回调
 * @param cb 回调函数
 */
void bluetooth_manager_set_health_check_callback(health_check_result_cb_t cb);

/**
 * @brief 设置故障报告回调
 * @param cb 回调函数
 */
void bluetooth_manager_set_fault_report_callback(fault_report_cb_t cb);

/* =================== 便利宏定义 =================== */

/* 快速颜色控制宏 */
#define BT_BROADCAST_RED_ALL()           bluetooth_manager_broadcast_all_set_color(LED_COLOR_RED, BLINK_MODE_NONE, 0)
#define BT_BROADCAST_GREEN_ALL()         bluetooth_manager_broadcast_all_set_color(LED_COLOR_GREEN, BLINK_MODE_NONE, 0)
#define BT_BROADCAST_BLUE_ALL()          bluetooth_manager_broadcast_all_set_color(LED_COLOR_BLUE, BLINK_MODE_NONE, 0)
#define BT_BROADCAST_OFF_ALL()           bluetooth_manager_broadcast_all_off()

/* 快速闪烁控制宏 */
#define BT_BROADCAST_BLINK_RED_ALL()     bluetooth_manager_broadcast_all_set_color(LED_COLOR_RED, BLINK_MODE_FAST, 0)
#define BT_BROADCAST_BLINK_YELLOW_ALL()  bluetooth_manager_broadcast_all_set_color(LED_COLOR_YELLOW, BLINK_MODE_SLOW, 0)

/* 快速分组控制宏 */
#define BT_BROADCAST_GROUP_RED(group)    bluetooth_manager_broadcast_group_set_color(group, LED_COLOR_RED, BLINK_MODE_NONE, 0)
#define BT_BROADCAST_GROUP_GREEN(group)  bluetooth_manager_broadcast_group_set_color(group, LED_COLOR_GREEN, BLINK_MODE_NONE, 0)

#endif // BLUETOOTH_MANAGER_H 