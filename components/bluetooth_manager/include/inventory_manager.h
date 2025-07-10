#ifndef INVENTORY_MANAGER_H
#define INVENTORY_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 盘点管理器错误代码 */
typedef enum {
    INVENTORY_ERR_OK = 0,
    INVENTORY_ERR_INVALID_PARAM,
    INVENTORY_ERR_NO_MEMORY,
    INVENTORY_ERR_TIMEOUT,
    INVENTORY_ERR_NOT_INITIALIZED,
    INVENTORY_ERR_SCAN_ACTIVE,
    INVENTORY_ERR_MAX
} inventory_err_t;

/* 盘点状态 */
typedef enum {
    INVENTORY_STATE_IDLE = 0,
    INVENTORY_STATE_SCANNING,
    INVENTORY_STATE_COLLECTING,
    INVENTORY_STATE_COMPLETED,
    INVENTORY_STATE_ERROR
} inventory_state_t;

/* 盘点统计信息 */
typedef struct {
    uint32_t total_tags;        // 总标签数
    uint32_t responding_tags;   // 响应标签数
    uint32_t low_battery_tags;  // 低电量标签数
    uint32_t faulty_tags;       // 故障标签数
    uint32_t scan_duration_ms;  // 扫描持续时间
} inventory_stats_t;

/* 盘点结果回调 */
typedef void (*inventory_complete_cb_t)(uint32_t scan_id, inventory_state_t state, const inventory_stats_t *stats);

/* 标签发现回调 */
typedef void (*tag_discovered_cb_t)(uint32_t scan_id, const inventory_info_t *tag_info);

/**
 * @brief 初始化盘点管理器
 * @return 错误代码
 */
inventory_err_t inventory_manager_init(void);

/**
 * @brief 反初始化盘点管理器
 */
void inventory_manager_deinit(void);

/**
 * @brief 开始全量盘点
 * @param scan_id 盘点ID
 * @param timeout_ms 超时时间(毫秒)
 * @return 错误代码
 */
inventory_err_t inventory_manager_start_full_scan(uint32_t scan_id, uint32_t timeout_ms);

/**
 * @brief 开始分组盘点
 * @param scan_id 盘点ID
 * @param group_id 分组ID
 * @param timeout_ms 超时时间(毫秒)
 * @return 错误代码
 */
inventory_err_t inventory_manager_start_group_scan(uint32_t scan_id, uint8_t group_id, uint32_t timeout_ms);

/**
 * @brief 停止当前盘点
 * @return 错误代码
 */
inventory_err_t inventory_manager_stop_scan(void);

/**
 * @brief 获取当前盘点状态
 * @return 盘点状态
 */
inventory_state_t inventory_manager_get_state(void);

/**
 * @brief 获取当前盘点统计信息
 * @param stats 统计信息输出缓冲区
 * @return 错误代码
 */
inventory_err_t inventory_manager_get_stats(inventory_stats_t *stats);

/**
 * @brief 设置盘点完成回调
 * @param cb 回调函数
 */
void inventory_manager_set_complete_callback(inventory_complete_cb_t cb);

/**
 * @brief 设置标签发现回调
 * @param cb 回调函数
 */
void inventory_manager_set_discovery_callback(tag_discovered_cb_t cb);

/**
 * @brief 处理标签响应数据
 * @param tag_id 标签ID
 * @param data 响应数据
 * @param length 数据长度
 * @return 错误代码
 */
inventory_err_t inventory_manager_handle_tag_response(uint32_t tag_id, const uint8_t *data, uint16_t length);

/**
 * @brief 获取标签信息
 * @param tag_id 标签ID
 * @param info 标签信息输出缓冲区
 * @return 错误代码
 */
inventory_err_t inventory_manager_get_tag_info(uint32_t tag_id, inventory_info_t *info);

/**
 * @brief 清除标签信息记录
 * @param tag_id 标签ID (0表示清除所有)
 * @return 错误代码
 */
inventory_err_t inventory_manager_clear_tag_info(uint32_t tag_id);

/**
 * @brief 设置标签故障状态
 * @param tag_id 标签ID
 * @param fault_flags 故障标志位
 * @return 错误代码
 */
inventory_err_t inventory_manager_set_tag_fault(uint32_t tag_id, uint8_t fault_flags);

/**
 * @brief 获取低电量标签列表
 * @param tag_ids 标签ID数组
 * @param max_count 最大数量
 * @param actual_count 实际数量输出
 * @return 错误代码
 */
inventory_err_t inventory_manager_get_low_battery_tags(uint32_t *tag_ids, uint16_t max_count, uint16_t *actual_count);

/**
 * @brief 获取故障标签列表
 * @param tag_ids 标签ID数组
 * @param max_count 最大数量
 * @param actual_count 实际数量输出
 * @return 错误代码
 */
inventory_err_t inventory_manager_get_faulty_tags(uint32_t *tag_ids, uint16_t max_count, uint16_t *actual_count);

#ifdef __cplusplus
}
#endif

#endif // INVENTORY_MANAGER_H