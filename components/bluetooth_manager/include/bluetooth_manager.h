#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include "common.h"
#include <stdint.h>
#include <stdbool.h>

/* 最大支持的灯条设备数量 */
#define MAX_LED_DEVICES             10
#define BLE_SCAN_DURATION_MS        10000  // 扫描持续时间10秒
#define BLE_CONNECT_TIMEOUT_MS      5000   // 连接超时5秒

/* 蓝牙管理器错误代码 */
typedef enum {
    BT_MGR_OK = 0,
    BT_MGR_ERR_INIT_FAILED = -1,
    BT_MGR_ERR_SCAN_FAILED = -2,
    BT_MGR_ERR_CONNECT_FAILED = -3,
    BT_MGR_ERR_NOT_INITIALIZED = -4,
    BT_MGR_ERR_DEVICE_NOT_FOUND = -5,
    BT_MGR_ERR_DEVICE_FULL = -6,
    BT_MGR_ERR_INVALID_PARAM = -7
} bt_mgr_err_t;

/* 蓝牙管理器状态 */
typedef enum {
    BT_MGR_STATE_UNINITIALIZED = 0,
    BT_MGR_STATE_INITIALIZED,
    BT_MGR_STATE_SCANNING,
    BT_MGR_STATE_CONNECTING,
    BT_MGR_STATE_READY
} bt_mgr_state_t;

/* 灯条设备连接状态 */
typedef enum {
    LED_DEVICE_STATE_DISCONNECTED = 0,
    LED_DEVICE_STATE_CONNECTING,
    LED_DEVICE_STATE_CONNECTED,
    LED_DEVICE_STATE_ERROR
} led_device_state_t;

/* 灯条设备信息结构 */
typedef struct {
    uint8_t device_id;                  // 设备ID (1-10)
    uint8_t mac_addr[6];               // MAC地址
    uint8_t addr_type;                 // 地址类型 (0=公共地址, 1=随机地址)
    char name[32];                     // 设备名称
    uint16_t conn_handle;              // 连接句柄
    led_device_state_t state;          // 连接状态
    int8_t rssi;                       // 信号强度
    uint32_t last_seen;                // 最后见到时间(ms)
    uint32_t last_response;            // 最后响应时间(ms)
    
    // GATT特征值句柄
    uint16_t cmd_chr_handle;           // 命令特征值句柄
    uint16_t notify_chr_handle;        // 通知特征值句柄
    uint16_t notify_desc_handle;       // 通知描述符句柄
    
    // 设备状态
    led_color_t current_color;         // 当前颜色
    bool is_on;                        // 是否开启
    uint8_t battery_level;             // 电池电量
} led_device_t;

/* 设备扫描结果结构 */
typedef struct {
    uint8_t mac_addr[6];
    uint8_t addr_type;                 // 地址类型 (0=公共地址, 1=随机地址)
    char name[32];
    int8_t rssi;
    bool is_led_device;                // 是否为灯条设备
} scan_result_t;

/* 设备发现回调函数类型 */
typedef void (*device_found_cb_t)(const scan_result_t *result);

/* 设备状态变化回调函数类型 */
typedef void (*device_state_cb_t)(uint8_t device_id, led_device_state_t old_state, led_device_state_t new_state);

/* 设备响应回调函数类型 */
typedef void (*device_response_cb_t)(uint8_t device_id, const uint8_t *data, uint16_t length);

/* 基站蓝牙管理器API */

/**
 * @brief 初始化蓝牙管理器(Central模式)
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_init(void);

/**
 * @brief 启动基站服务(开始扫描设备)
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_start(void);

/**
 * @brief 停止基站服务
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_stop(void);

/**
 * @brief 获取管理器状态
 * @return 当前状态
 */
bt_mgr_state_t bluetooth_manager_get_state(void);

/**
 * @brief 开始扫描灯条设备
 * @param duration_ms 扫描持续时间(毫秒)，0表示持续扫描
 * @param found_cb 设备发现回调函数
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_start_scan(uint32_t duration_ms, device_found_cb_t found_cb);

/**
 * @brief 停止扫描
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_stop_scan(void);

/**
 * @brief 连接灯条设备
 * @param mac_addr 设备MAC地址
 * @param device_id 分配的设备ID (1-10)
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_connect_device(const uint8_t *mac_addr, uint8_t device_id);

/**
 * @brief 连接灯条设备（指定地址类型）
 * @param mac_addr 设备MAC地址
 * @param device_id 分配的设备ID (1-10)
 * @param addr_type 地址类型 (0=公共地址, 1=随机地址)
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_connect_device_with_type(const uint8_t *mac_addr, uint8_t device_id, uint8_t addr_type);

/**
 * @brief 断开灯条设备
 * @param device_id 设备ID
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_disconnect_device(uint8_t device_id);

/**
 * @brief 发送命令到指定灯条
 * @param device_id 设备ID
 * @param cmd 命令结构
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_send_command(uint8_t device_id, const led_command_t *cmd);

/**
 * @brief 广播命令到所有已连接灯条
 * @param cmd 命令结构
 * @return 成功发送的设备数量
 */
int bluetooth_manager_broadcast_command(const led_command_t *cmd);

/**
 * @brief 获取设备信息
 * @param device_id 设备ID
 * @return 设备信息指针，失败返回NULL
 */
const led_device_t* bluetooth_manager_get_device(uint8_t device_id);

/**
 * @brief 获取所有已连接设备列表
 * @param devices 设备数组缓冲区
 * @param max_count 最大设备数量
 * @return 实际连接的设备数量
 */
int bluetooth_manager_get_connected_devices(led_device_t *devices, int max_count);

/**
 * @brief 获取已连接设备数量
 * @return 连接设备数量
 */
int bluetooth_manager_get_connected_count(void);

/**
 * @brief 设置设备状态变化回调
 * @param cb 回调函数
 */
void bluetooth_manager_set_state_callback(device_state_cb_t cb);

/**
 * @brief 设置设备响应回调
 * @param cb 回调函数
 */
void bluetooth_manager_set_response_callback(device_response_cb_t cb);

/**
 * @brief 检查设备是否在线
 * @param device_id 设备ID
 * @return true=在线，false=离线
 */
bool bluetooth_manager_is_device_online(uint8_t device_id);

/**
 * @brief 获取设备信号强度
 * @param device_id 设备ID
 * @return RSSI值(dBm)，错误返回-128
 */
int8_t bluetooth_manager_get_device_rssi(uint8_t device_id);

/* ========== 仓库管理系统扩展API ========== */

/**
 * @brief 激活指定编号的标签(LED+蜂鸣器)
 * @param warehouse_id 仓库位置编号 (对应device_id)
 * @param color LED颜色
 * @param duration_ms 激活持续时间(毫秒)
 * @param enable_beep 是否启用蜂鸣器
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_activate_tag(uint8_t warehouse_id, led_color_t color, 
                                           uint32_t duration_ms, bool enable_beep);

/**
 * @brief 停止指定标签的激活状态
 * @param warehouse_id 仓库位置编号
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_deactivate_tag(uint8_t warehouse_id);

/**
 * @brief 批量激活多个标签(用于盘点)
 * @param warehouse_ids 仓库位置编号数组
 * @param count 数组长度
 * @param color LED颜色
 * @param enable_beep 是否启用蜂鸣器
 * @return 成功激活的标签数量
 */
int bluetooth_manager_activate_multiple_tags(const uint8_t *warehouse_ids, int count, 
                                            led_color_t color, bool enable_beep);

/**
 * @brief 设置标签蜂鸣器状态
 * @param warehouse_id 仓库位置编号
 * @param enable 是否启用蜂鸣器
 * @return 错误代码
 */
bt_mgr_err_t bluetooth_manager_set_beep_state(uint8_t warehouse_id, bool enable);

/**
 * @brief 获取标签电池状态
 * @param warehouse_id 仓库位置编号
 * @return 电池电量等级 (0-4)，错误返回-1
 */
int bluetooth_manager_get_battery_level(uint8_t warehouse_id);

/**
 * @brief 执行仓库盘点 - 获取所有标签状态
 * @param inventory_report 盘点报告缓冲区
 * @param max_tags 最大标签数量
 * @return 实际盘点的标签数量
 */
int bluetooth_manager_inventory_check(led_device_t *inventory_report, int max_tags);

/**
 * @brief 检查低电量标签
 * @param low_battery_tags 低电量标签ID数组
 * @param max_count 最大数组长度
 * @return 低电量标签数量
 */
int bluetooth_manager_check_low_battery_tags(uint8_t *low_battery_tags, int max_count);

/**
 * @brief 广播紧急信号到所有标签(红灯+蜂鸣)
 * @return 成功激活的标签数量
 */
int bluetooth_manager_emergency_alert(void);

/* 仓库管理特定回调函数类型 */

/**
 * @brief 电池电量警报回调
 * @param warehouse_id 仓库位置编号
 * @param battery_level 电池电量等级
 */
typedef void (*battery_alert_cb_t)(uint8_t warehouse_id, uint8_t battery_level);

/**
 * @brief 设置电池电量警报回调
 * @param cb 回调函数
 */
void bluetooth_manager_set_battery_alert_callback(battery_alert_cb_t cb);

#endif // BLUETOOTH_MANAGER_H 