#include "system_config.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include <string.h>

// 全局变量定义
system_status_t g_system_status = {0};

void system_config_init(void)
{
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 命令队列将在main.c中创建

    // 初始化系统状态
    memset(&g_system_status, 0, sizeof(system_status_t));
    
    ESP_LOGI(STATION_TAG, "System configuration initialized");
}
