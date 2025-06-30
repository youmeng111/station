#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include "system_config.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "esp_netif.h"

// MQTT状态回调类型
typedef void (*mqtt_connection_callback_t)(bool connected);
typedef void (*mqtt_message_callback_t)(const char *topic, const char *data, int data_len);

// WiFi和MQTT管理器初始化
esp_err_t mqtt_manager_init(void);

// WiFi连接
esp_err_t wifi_connect(void);

// MQTT连接
esp_err_t mqtt_connect(void);

// 设置回调函数
void mqtt_set_connection_callback(mqtt_connection_callback_t callback);
void mqtt_set_message_callback(mqtt_message_callback_t callback);

// 发布消息
esp_err_t mqtt_publish(const char *topic, const char *data, int qos, int retain);

// 订阅主题
esp_err_t mqtt_subscribe(const char *topic, int qos);

// 发送系统状态
esp_err_t mqtt_send_status(void);

// 发送设备信息
esp_err_t mqtt_send_device_info(void);

// 处理收到的MQTT消息
void mqtt_handle_message(const char *topic, const char *data, int data_len);

// 检查连接状态
bool mqtt_is_connected(void);
bool wifi_is_connected(void);

// 断开连接
esp_err_t mqtt_disconnect(void);
esp_err_t wifi_disconnect(void);

// 重连功能
esp_err_t mqtt_manager_reconnect(void);

#endif // MQTT_MANAGER_H 