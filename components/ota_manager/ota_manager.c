#include "ota_manager.h"
#include "esp_log.h"
#include "esp_app_format.h"
#include "esp_partition.h"
#include <string.h>

static const char *TAG = "OTA_MANAGER";

static ota_status_t current_status = OTA_STATUS_IDLE;
static int current_progress = 0;
static char ota_url[OTA_URL_SIZE] = {0};
static bool ota_task_running = false;

static ota_progress_callback_t progress_callback = NULL;
static ota_status_callback_t status_callback = NULL;

static esp_ota_handle_t ota_handle = 0;
static const esp_partition_t *ota_partition = NULL;

// 设置OTA状态
static void set_ota_status(ota_status_t status, const char *message)
{
    current_status = status;
    ESP_LOGI(TAG, "OTA Status: %d, Message: %s", status, message ? message : "");
    
    if (status_callback) {
        status_callback(status, message);
    }
}

// 设置OTA进度
static void set_ota_progress(int percent)
{
    current_progress = percent;
    ESP_LOGI(TAG, "OTA Progress: %d%%", percent);
    
    if (progress_callback) {
        progress_callback(percent);
    }
}

// HTTP事件处理
static esp_err_t ota_http_event_handler(esp_http_client_event_t *evt)
{
    static int output_len = 0;
    static int total_len = 0;
    
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
        break;
        
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        output_len = 0;
        total_len = 0;
        break;
        
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
        
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        if (strcasecmp(evt->header_key, "Content-Length") == 0) {
            total_len = atoi(evt->header_value);
            ESP_LOGI(TAG, "Total firmware size: %d bytes", total_len);
        }
        break;
        
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        if (!esp_http_client_is_chunked_response(evt->client)) {
            if (ota_partition && ota_handle) {
                esp_err_t err = esp_ota_write(ota_handle, (const void *)evt->data, evt->data_len);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
                    return ESP_FAIL;
                }
                
                output_len += evt->data_len;
                if (total_len > 0) {
                    int progress = (output_len * 100) / total_len;
                    set_ota_progress(progress);
                }
            }
        }
        break;
        
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
        break;
        
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
        
    case HTTP_EVENT_REDIRECT:
        ESP_LOGI(TAG, "HTTP_EVENT_REDIRECT");
        break;
        
    default:
        ESP_LOGW(TAG, "Unhandled HTTP event: %d", evt->event_id);
        break;
    }
    return ESP_OK;
}

esp_err_t ota_manager_init(void)
{
    ESP_LOGI(TAG, "OTA Manager initialized");
    return ESP_OK;
}

void ota_set_progress_callback(ota_progress_callback_t callback)
{
    progress_callback = callback;
}

void ota_set_status_callback(ota_status_callback_t callback)
{
    status_callback = callback;
}

esp_err_t ota_start_update(const char *url)
{
    if (ota_task_running) {
        ESP_LOGW(TAG, "OTA task already running");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!url || strlen(url) >= OTA_URL_SIZE) {
        ESP_LOGE(TAG, "Invalid OTA URL");
        return ESP_ERR_INVALID_ARG;
    }
    
    strncpy(ota_url, url, sizeof(ota_url) - 1);
    ota_url[sizeof(ota_url) - 1] = '\0';
    
    // 创建OTA任务
    BaseType_t ret = xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create OTA task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "OTA update started for URL: %s", url);
    return ESP_OK;
}

ota_status_t ota_get_status(void)
{
    return current_status;
}

int ota_get_progress(void)
{
    return current_progress;
}

esp_err_t ota_cancel_update(void)
{
    if (current_status == OTA_STATUS_DOWNLOADING || current_status == OTA_STATUS_UPDATING) {
        set_ota_status(OTA_STATUS_FAILED, "Update cancelled by user");
        ota_task_running = false;
        return ESP_OK;
    }
    
    return ESP_ERR_INVALID_STATE;
}

esp_err_t ota_get_partition_info(char *buffer, size_t buffer_size)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    
    if (running && boot) {
        const esp_app_desc_t *app_desc = esp_app_get_description();
        snprintf(buffer, buffer_size,
                 "{\"running_partition\":\"%s\",\"boot_partition\":\"%s\",\"app_version\":\"%s\"}",
                 running->label, boot->label, app_desc ? app_desc->version : "unknown");
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t ota_restart_to_new_firmware(void)
{
    if (current_status == OTA_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Restarting to new firmware...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
    
    return ESP_ERR_INVALID_STATE;
}

void ota_task(void *pvParameters)
{
    ota_task_running = true;
    current_progress = 0;
    
    set_ota_status(OTA_STATUS_DOWNLOADING, "Starting download");
    
    // 获取下一个可用的OTA分区
    ota_partition = esp_ota_get_next_update_partition(NULL);
    if (ota_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition found");
        set_ota_status(OTA_STATUS_FAILED, "No OTA partition found");
        goto exit;
    }
    
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%lx",
             ota_partition->subtype, ota_partition->address);
    
    // 开始OTA升级
    esp_err_t err = esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        set_ota_status(OTA_STATUS_FAILED, "OTA begin failed");
        goto exit;
    }
    
    // 配置HTTP客户端
    esp_http_client_config_t config = {
        .url = ota_url,
        .event_handler = ota_http_event_handler,
        .timeout_ms = OTA_RECEIVE_TIMEOUT,
        .keep_alive_enable = true,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        set_ota_status(OTA_STATUS_FAILED, "HTTP client init failed");
        goto cleanup_ota;
    }
    
    // 执行HTTP请求
    err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP client perform failed: %s", esp_err_to_name(err));
        set_ota_status(OTA_STATUS_FAILED, "Download failed");
        goto cleanup_http;
    }
    
    // 检查HTTP状态码
    int status_code = esp_http_client_get_status_code(client);
    if (status_code != 200) {
        ESP_LOGE(TAG, "HTTP status code: %d", status_code);
        set_ota_status(OTA_STATUS_FAILED, "HTTP error");
        goto cleanup_http;
    }
    
    set_ota_status(OTA_STATUS_UPDATING, "Finalizing update");
    
    // 完成OTA升级
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
            set_ota_status(OTA_STATUS_FAILED, "Image validation failed");
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
            set_ota_status(OTA_STATUS_FAILED, "OTA end failed");
        }
        goto cleanup_http;
    }
    
    // 设置启动分区
    err = esp_ota_set_boot_partition(ota_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        set_ota_status(OTA_STATUS_FAILED, "Set boot partition failed");
        goto cleanup_http;
    }
    
    set_ota_progress(100);
    set_ota_status(OTA_STATUS_SUCCESS, "Update completed successfully");
    ESP_LOGI(TAG, "OTA update successful. Restart required.");
    
cleanup_http:
    esp_http_client_cleanup(client);
    
cleanup_ota:
    if (ota_handle) {
        esp_ota_abort(ota_handle);
    }
    
exit:
    ota_task_running = false;
    vTaskDelete(NULL);
} 