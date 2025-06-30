#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include "system_config.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"

// OTA状态枚举
typedef enum {
    OTA_STATUS_IDLE = 0,
    OTA_STATUS_DOWNLOADING,
    OTA_STATUS_UPDATING,
    OTA_STATUS_SUCCESS,
    OTA_STATUS_FAILED
} ota_status_t;

// OTA进度回调类型
typedef void (*ota_progress_callback_t)(int percent);
typedef void (*ota_status_callback_t)(ota_status_t status, const char *message);

// OTA管理器初始化
esp_err_t ota_manager_init(void);

// 设置回调函数
void ota_set_progress_callback(ota_progress_callback_t callback);
void ota_set_status_callback(ota_status_callback_t callback);

// 开始OTA升级
esp_err_t ota_start_update(const char *url);

// 获取当前OTA状态
ota_status_t ota_get_status(void);

// 获取OTA进度
int ota_get_progress(void);

// 取消OTA升级
esp_err_t ota_cancel_update(void);

// 检查分区信息
esp_err_t ota_get_partition_info(char *buffer, size_t buffer_size);

// 重启到新固件
esp_err_t ota_restart_to_new_firmware(void);

// OTA任务
void ota_task(void *pvParameters);

#endif // OTA_MANAGER_H 