#include "led_controller.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "LED_CONTROLLER";
static led_strip_state_t led_strips[MAX_LED_STRIPS];
static QueueHandle_t led_command_queue = NULL;

// LED定时器回调函数
static void led_timer_callback(void* arg)
{
    uint8_t strip_id = (uint8_t)(uintptr_t)arg;
    
    if (strip_id < MAX_LED_STRIPS) {
        ESP_LOGI(TAG, "LED Strip %d timer expired, turning off", strip_id);
        led_strip_off(strip_id);
    }
}

esp_err_t led_controller_init(QueueHandle_t cmd_queue)
{
    led_command_queue = cmd_queue;
    
    // 初始化所有LED条状态
    for (int i = 0; i < MAX_LED_STRIPS; i++) {
        led_strips[i].is_on = false;
        led_strips[i].color = LED_COLOR_OFF;
        led_strips[i].duration_ms = 0;
        led_strips[i].start_time_ms = 0;
        led_strips[i].timer_handle = NULL;
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
    led_strips[strip_id].color = LED_COLOR_WHITE; // 默认白色
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

    // 停止定时器
    if (led_strips[strip_id].timer_handle != NULL) {
        esp_timer_stop((esp_timer_handle_t)led_strips[strip_id].timer_handle);
        esp_timer_delete((esp_timer_handle_t)led_strips[strip_id].timer_handle);
        led_strips[strip_id].timer_handle = NULL;
    }

    led_strips[strip_id].is_on = false;
    led_strips[strip_id].color = LED_COLOR_OFF;
    led_strips[strip_id].duration_ms = 0;
    led_strips[strip_id].start_time_ms = 0;
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

esp_err_t led_strip_set_color(uint8_t strip_id, led_color_t color)
{
    if (strip_id >= MAX_LED_STRIPS) {
        ESP_LOGE(TAG, "Invalid strip ID: %d", strip_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (color == LED_COLOR_OFF) {
        return led_strip_off(strip_id);
    }

    led_strips[strip_id].color = color;
    led_strips[strip_id].is_on = true;
    g_system_status.led_status[strip_id] = 1;

    ESP_LOGI(TAG, "LED Strip %d color set to %s", strip_id, led_color_to_string(color));
    return ESP_OK;
}

esp_err_t led_strip_set_color_with_duration(uint8_t strip_id, led_color_t color, uint32_t duration_ms)
{
    if (strip_id >= MAX_LED_STRIPS) {
        ESP_LOGE(TAG, "Invalid strip ID: %d", strip_id);
        return ESP_ERR_INVALID_ARG;
    }

    // 先设置颜色
    esp_err_t ret = led_strip_set_color(strip_id, color);
    if (ret != ESP_OK) {
        return ret;
    }

    // 如果之前有定时器，先停止并删除
    if (led_strips[strip_id].timer_handle != NULL) {
        esp_timer_stop((esp_timer_handle_t)led_strips[strip_id].timer_handle);
        esp_timer_delete((esp_timer_handle_t)led_strips[strip_id].timer_handle);
        led_strips[strip_id].timer_handle = NULL;
    }

    // 如果duration_ms为0，表示永久点亮
    if (duration_ms == 0) {
        led_strips[strip_id].duration_ms = 0;
        led_strips[strip_id].start_time_ms = 0;
        return ESP_OK;
    }

    // 创建定时器
    esp_timer_create_args_t timer_args = {
        .callback = led_timer_callback,
        .arg = (void*)(uintptr_t)strip_id,
        .name = "led_timer"
    };

    esp_timer_handle_t timer_handle;
    ret = esp_timer_create(&timer_args, &timer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer for LED strip %d", strip_id);
        return ret;
    }

    // 启动定时器
    ret = esp_timer_start_once(timer_handle, duration_ms * 1000); // 转换为微秒
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer for LED strip %d", strip_id);
        esp_timer_delete(timer_handle);
        return ret;
    }

    led_strips[strip_id].timer_handle = timer_handle;

    led_strips[strip_id].duration_ms = duration_ms;
    led_strips[strip_id].start_time_ms = esp_timer_get_time() / 1000; // 转换为毫秒

    ESP_LOGI(TAG, "LED Strip %d set to %s for %ld ms", strip_id, led_color_to_string(color), duration_ms);
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
                    
                case CMD_LED_COLOR:
                    led_strip_set_color(command.led_id, (led_color_t)command.param1);
                    break;
                    
                case CMD_LED_COLOR_WITH_TIME:
                    {
                        uint32_t duration_ms = ((uint32_t)command.param3 << 16) | command.param2;
                        led_strip_set_color_with_duration(command.led_id, (led_color_t)command.param1, duration_ms);
                    }
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
        
        uint32_t remaining_time = 0;
        if (led_strips[i].is_on && led_strips[i].duration_ms > 0) {
            uint32_t current_time = esp_timer_get_time() / 1000;
            uint32_t elapsed = current_time - led_strips[i].start_time_ms;
            if (elapsed < led_strips[i].duration_ms) {
                remaining_time = led_strips[i].duration_ms - elapsed;
            }
        }
        
        offset += snprintf(buffer + offset, buffer_size - offset,
                          "{\"id\":%d,\"on\":%s,\"color\":\"%s\",\"duration\":%ld,\"remaining\":%ld}",
                          i,
                          led_strips[i].is_on ? "true" : "false",
                          led_color_to_string(led_strips[i].color),
                          led_strips[i].duration_ms,
                          remaining_time);
    }
    
    offset += snprintf(buffer + offset, buffer_size - offset, "]}");
    
    return offset;
}

// 蓝牙管理器需要的函数接口实现
esp_err_t led_controller_turn_on(uint8_t led_id)
{
    return led_strip_on(led_id);
}

esp_err_t led_controller_turn_off(uint8_t led_id)
{
    return led_strip_off(led_id);
}

esp_err_t led_controller_set_color_simple(uint8_t led_id, led_color_t color)
{
    return led_strip_set_color(led_id, color);
}

esp_err_t led_controller_set_color_with_time(uint8_t led_id, led_color_t color, uint32_t duration_ms)
{
    return led_strip_set_color_with_duration(led_id, color, duration_ms);
}

bool led_controller_is_on(uint8_t led_id)
{
    if (led_id >= MAX_LED_STRIPS) {
        return false;
    }
    return led_strips[led_id].is_on;
}

led_color_t led_controller_get_color(uint8_t led_id)
{
    if (led_id >= MAX_LED_STRIPS) {
        return LED_COLOR_OFF;
    }
    return led_strips[led_id].color;
}

// 颜色名称转换函数
led_color_t led_color_from_string(const char* color_str)
{
    if (color_str == NULL) {
        return LED_COLOR_OFF;
    }
    
    if (strcasecmp(color_str, "RED") == 0) {
        return LED_COLOR_RED;
    } else if (strcasecmp(color_str, "GREEN") == 0) {
        return LED_COLOR_GREEN;
    } else if (strcasecmp(color_str, "BLUE") == 0) {
        return LED_COLOR_BLUE;
    } else if (strcasecmp(color_str, "YELLOW") == 0) {
        return LED_COLOR_YELLOW;
    } else if (strcasecmp(color_str, "PURPLE") == 0) {
        return LED_COLOR_PURPLE;
    } else if (strcasecmp(color_str, "CYAN") == 0) {
        return LED_COLOR_CYAN;
    } else if (strcasecmp(color_str, "WHITE") == 0) {
        return LED_COLOR_WHITE;
    } else if (strcasecmp(color_str, "OFF") == 0) {
        return LED_COLOR_OFF;
    }
    
    return LED_COLOR_OFF;
}

const char* led_color_to_string(led_color_t color)
{
    switch (color) {
        case LED_COLOR_RED:    return "RED";
        case LED_COLOR_GREEN:  return "GREEN";
        case LED_COLOR_BLUE:   return "BLUE";
        case LED_COLOR_YELLOW: return "YELLOW";
        case LED_COLOR_PURPLE: return "PURPLE";
        case LED_COLOR_CYAN:   return "CYAN";
        case LED_COLOR_WHITE:  return "WHITE";
        case LED_COLOR_OFF:    return "OFF";
        default:               return "UNKNOWN";
    }
} 