#include "led_controller.h"
#include "system_config.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "LED_CONTROLLER";
static led_strip_state_t led_strips[MAX_LED_STRIPS];
static QueueHandle_t led_command_queue = NULL;

esp_err_t led_controller_init(QueueHandle_t cmd_queue)
{
    led_command_queue = cmd_queue;
    
    // 初始化所有LED条状态
    for (int i = 0; i < MAX_LED_STRIPS; i++) {
        led_strips[i].is_on = false;
        led_strips[i].brightness = 128;  // 默认中等亮度
        led_strips[i].red = 255;
        led_strips[i].green = 255;
        led_strips[i].blue = 255;
    }

    // 创建LED命令处理任务
    xTaskCreate(led_command_handler_task, "led_cmd_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "LED Controller initialized with %d strips", MAX_LED_STRIPS);
    return ESP_OK;
}

esp_err_t led_strip_on(uint8_t strip_id)
{
    if (strip_id >= MAX_LED_STRIPS) {
        ESP_LOGE(TAG, "Invalid strip ID: %d", strip_id);
        return ESP_ERR_INVALID_ARG;
    }

    led_strips[strip_id].is_on = true;
    g_system_status.led_status[strip_id] = 1;

    ESP_LOGI(TAG, "LED Strip %d turned ON", strip_id);
    return ESP_OK;
}

esp_err_t led_strip_off(uint8_t strip_id)
{
    if (strip_id >= MAX_LED_STRIPS) {
        ESP_LOGE(TAG, "Invalid strip ID: %d", strip_id);
        return ESP_ERR_INVALID_ARG;
    }

    led_strips[strip_id].is_on = false;
    g_system_status.led_status[strip_id] = 0;

    ESP_LOGI(TAG, "LED Strip %d turned OFF", strip_id);
    return ESP_OK;
}

esp_err_t led_strip_toggle(uint8_t strip_id)
{
    if (strip_id >= MAX_LED_STRIPS) {
        ESP_LOGE(TAG, "Invalid strip ID: %d", strip_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (led_strips[strip_id].is_on) {
        return led_strip_off(strip_id);
    } else {
        return led_strip_on(strip_id);
    }
}

esp_err_t led_strip_set_brightness(uint8_t strip_id, uint8_t brightness)
{
    if (strip_id >= MAX_LED_STRIPS) {
        ESP_LOGE(TAG, "Invalid strip ID: %d", strip_id);
        return ESP_ERR_INVALID_ARG;
    }

    led_strips[strip_id].brightness = brightness;

    ESP_LOGI(TAG, "LED Strip %d brightness set to %d", strip_id, brightness);
    return ESP_OK;
}

esp_err_t led_strip_set_color(uint8_t strip_id, uint8_t r, uint8_t g, uint8_t b)
{
    if (strip_id >= MAX_LED_STRIPS) {
        ESP_LOGE(TAG, "Invalid strip ID: %d", strip_id);
        return ESP_ERR_INVALID_ARG;
    }

    led_strips[strip_id].red = r;
    led_strips[strip_id].green = g;
    led_strips[strip_id].blue = b;

    ESP_LOGI(TAG, "LED Strip %d color set to RGB(%d,%d,%d)", strip_id, r, g, b);
    return ESP_OK;
}

led_strip_state_t* led_get_state(uint8_t strip_id)
{
    if (strip_id >= MAX_LED_STRIPS) {
        return NULL;
    }
    return &led_strips[strip_id];
}

void led_command_handler_task(void *pvParameters)
{
    led_command_t command;
    
    ESP_LOGI(TAG, "LED Command Handler Task started");
    
    while (1) {
        if (xQueueReceive(led_command_queue, &command, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Processing command: type=%d, led_id=%d", command.type, command.led_id);
            
            switch (command.type) {
                case CMD_LED_ON:
                    led_strip_on(command.led_id);
                    break;
                    
                case CMD_LED_OFF:
                    led_strip_off(command.led_id);
                    break;
                    
                case CMD_LED_TOGGLE:
                    led_strip_toggle(command.led_id);
                    break;
                    
                case CMD_LED_BRIGHTNESS:
                    led_strip_set_brightness(command.led_id, command.param1);
                    break;
                    
                case CMD_LED_COLOR:
                    led_strip_set_color(command.led_id, command.param1, command.param2, command.param3);
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown command type: %d", command.type);
                    break;
            }
        }
    }
}

int led_get_status_json(char *buffer, size_t buffer_size)
{
    int offset = 0;
    
    offset += snprintf(buffer + offset, buffer_size - offset, "{\"led_strips\":[");
    
    for (int i = 0; i < MAX_LED_STRIPS; i++) {
        if (i > 0) {
            offset += snprintf(buffer + offset, buffer_size - offset, ",");
        }
        
        offset += snprintf(buffer + offset, buffer_size - offset,
                          "{\"id\":%d,\"on\":%s,\"brightness\":%d,\"color\":{\"r\":%d,\"g\":%d,\"b\":%d}}",
                          i,
                          led_strips[i].is_on ? "true" : "false",
                          led_strips[i].brightness,
                          led_strips[i].red,
                          led_strips[i].green,
                          led_strips[i].blue);
    }
    
    offset += snprintf(buffer + offset, buffer_size - offset, "]}");
    
    return offset;
} 