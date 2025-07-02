#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "system_config.h"

// LED颜色定义
typedef enum {
    LED_COLOR_OFF = 0,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,
    LED_COLOR_YELLOW,
    LED_COLOR_PURPLE,
    LED_COLOR_CYAN,
    LED_COLOR_WHITE
} led_color_t;

// LED状态定义
typedef struct {
    bool is_on;
    led_color_t color;
    uint32_t duration_ms;        // 持续时间（毫秒），0表示永久
    uint32_t start_time_ms;      // 开始时间戳
    void* timer_handle;          // 定时器句柄（使用void*避免头文件依赖）
} led_strip_state_t;

// LED控制器初始化
esp_err_t led_controller_init(QueueHandle_t cmd_queue);

// LED控制函数
esp_err_t led_strip_on(uint8_t strip_id);
esp_err_t led_strip_off(uint8_t strip_id);
esp_err_t led_strip_toggle(uint8_t strip_id);
esp_err_t led_strip_set_color(uint8_t strip_id, led_color_t color);
esp_err_t led_strip_set_color_with_duration(uint8_t strip_id, led_color_t color, uint32_t duration_ms);

// 获取LED状态
led_strip_state_t* led_get_state(uint8_t strip_id);

// 处理LED命令
void led_command_handler_task(void *pvParameters);

// 获取所有LED状态的JSON字符串
int led_get_status_json(char *buffer, size_t buffer_size);

// 蓝牙管理器需要的函数接口
esp_err_t led_controller_turn_on(uint8_t led_id);
esp_err_t led_controller_turn_off(uint8_t led_id);
esp_err_t led_controller_set_color_simple(uint8_t led_id, led_color_t color);
esp_err_t led_controller_set_color_with_time(uint8_t led_id, led_color_t color, uint32_t duration_ms);
bool led_controller_is_on(uint8_t led_id);
led_color_t led_controller_get_color(uint8_t led_id);

// 颜色名称转换函数
led_color_t led_color_from_string(const char* color_str);
const char* led_color_to_string(led_color_t color);

#endif // LED_CONTROLLER_H 