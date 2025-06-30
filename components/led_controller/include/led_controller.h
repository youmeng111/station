#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "system_config.h"

// LED状态定义
typedef struct {
    bool is_on;
    uint8_t brightness;  // 0-255
    uint8_t red;        // 0-255
    uint8_t green;      // 0-255
    uint8_t blue;       // 0-255
} led_strip_state_t;

// LED控制器初始化
esp_err_t led_controller_init(QueueHandle_t cmd_queue);

// LED控制函数
esp_err_t led_strip_on(uint8_t strip_id);
esp_err_t led_strip_off(uint8_t strip_id);
esp_err_t led_strip_toggle(uint8_t strip_id);
esp_err_t led_strip_set_brightness(uint8_t strip_id, uint8_t brightness);
esp_err_t led_strip_set_color(uint8_t strip_id, uint8_t r, uint8_t g, uint8_t b);

// 获取LED状态
led_strip_state_t* led_get_state(uint8_t strip_id);

// 处理LED命令
void led_command_handler_task(void *pvParameters);

// 获取所有LED状态的JSON字符串
int led_get_status_json(char *buffer, size_t buffer_size);

#endif // LED_CONTROLLER_H 