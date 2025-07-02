#include "mqtt_manager.h"
#include "system_config.h"
#include "led_controller.h"
#include "esp_log.h"
#include "esp_event.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "MQTT_MANAGER";

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool wifi_connected = false;
static bool mqtt_connected = false;
static int wifi_retry_count = 0;

static mqtt_connection_callback_t mqtt_connection_callback = NULL;
static mqtt_message_callback_t mqtt_message_callback = NULL;

// WiFi事件处理
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            wifi_retry_count++;
            ESP_LOGI(TAG, "retry to connect to the AP, count: %d", wifi_retry_count);
        } else {
            ESP_LOGI(TAG, "connect to the AP fail");
        }
        wifi_connected = false;
        g_system_status.wifi_connected = false;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0;
        wifi_connected = true;
        g_system_status.wifi_connected = true;
        
        // WiFi连接成功后自动连接MQTT
        mqtt_connect();
    }
}

// MQTT事件处理
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%lu", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        mqtt_connected = true;
        g_system_status.mqtt_connected = true;
        
        if (mqtt_connection_callback) {
            mqtt_connection_callback(true);
        }
        
        // 订阅控制主题
        mqtt_subscribe(MQTT_TOPIC_COMMAND, 1);
        
        // 发送设备上线消息
        mqtt_send_device_info();
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        g_system_status.mqtt_connected = false;
        
        if (mqtt_connection_callback) {
            mqtt_connection_callback(false);
        }
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        
        // 处理接收到的消息
        char topic[128] = {0};
        char data[256] = {0};
        
        if (event->topic_len < sizeof(topic)) {
            memcpy(topic, event->topic, event->topic_len);
        }
        if (event->data_len < sizeof(data)) {
            memcpy(data, event->data, event->data_len);
        }
        
        mqtt_handle_message(topic, data, event->data_len);
        
        if (mqtt_message_callback) {
            mqtt_message_callback(topic, data, event->data_len);
        }
        break;
        
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
        
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

esp_err_t mqtt_manager_init(void)
{
    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_LOGI(TAG, "MQTT Manager initialized");
    return ESP_OK;
}

esp_err_t wifi_connect(void)
{
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", WIFI_SSID, WIFI_PASSWORD);

    return ESP_OK;
}

esp_err_t mqtt_connect(void)
{
    if (!wifi_connected) {
        ESP_LOGW(TAG, "WiFi not connected, cannot connect to MQTT");
        return ESP_ERR_INVALID_STATE;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
        .broker.address.port = MQTT_PORT,
        .credentials.client_id = MQTT_CLIENT_ID,
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,
        .session.keepalive = 60,
        .session.disable_clean_session = false,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    ESP_LOGI(TAG, "MQTT client started");
    return ESP_OK;
}

void mqtt_set_connection_callback(mqtt_connection_callback_t callback)
{
    mqtt_connection_callback = callback;
}

void mqtt_set_message_callback(mqtt_message_callback_t callback)
{
    mqtt_message_callback = callback;
}

esp_err_t mqtt_publish(const char *topic, const char *data, int qos, int retain)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT not connected");
        return ESP_ERR_INVALID_STATE;
    }

    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, data, 0, qos, retain);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d, topic=%s", msg_id, topic);

    return ESP_OK;
}

esp_err_t mqtt_subscribe(const char *topic, int qos)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT not connected");
        return ESP_ERR_INVALID_STATE;
    }

    int msg_id = esp_mqtt_client_subscribe(mqtt_client, topic, qos);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d, topic=%s", msg_id, topic);

    return ESP_OK;
}

esp_err_t mqtt_send_status(void)
{
    char status_buffer[512];
    int len = led_get_status_json(status_buffer, sizeof(status_buffer) - 50);
    
    // 添加系统状态信息
    char *pos = status_buffer + len - 1; // 移除最后的 '}'
    int remaining = sizeof(status_buffer) - len;
    
    snprintf(pos, remaining, 
             ",\"system\":{\"wifi\":%s,\"mqtt\":%s,\"bt\":%s,\"uptime\":%lu}}",
             g_system_status.wifi_connected ? "true" : "false",
             g_system_status.mqtt_connected ? "true" : "false", 
             g_system_status.bt_connected ? "true" : "disabled",  // 蓝牙功能已关闭
             g_system_status.uptime_seconds);
    
    return mqtt_publish(MQTT_TOPIC_STATUS, status_buffer, 1, 0);
}

esp_err_t mqtt_send_device_info(void)
{
    char info_buffer[256];
    snprintf(info_buffer, sizeof(info_buffer),
             "{\"device\":\"ESP32_Station\",\"version\":\"1.0\",\"max_strips\":%d,\"status\":\"online\"}",
             MAX_LED_STRIPS);
    
    return mqtt_publish(MQTT_TOPIC_RESPONSE, info_buffer, 1, 1);
}

void mqtt_handle_message(const char *topic, const char *data, int data_len)
{
    ESP_LOGI(TAG, "Handling MQTT message on topic: %s", topic);
    
    if (strcmp(topic, MQTT_TOPIC_COMMAND) == 0) {
        // 解析JSON命令格式: 
        // {"cmd":"LED_CONTROL","led_id":1,"action":"ON","color":"RED","duration":5000}
        // 或 {"cmd":"LED_OFF","led_id":1}
        
        led_command_t command = {0};
        
        // 简单的JSON解析（实际项目中建议使用cJSON库）
        char *cmd_start = strstr(data, "\"cmd\":\"");
        char *led_id_start = strstr(data, "\"led_id\":");
        
        if (cmd_start && led_id_start) {
            cmd_start += 7; // 跳过 "cmd":"
            char *cmd_end = strchr(cmd_start, '"');
            
            led_id_start += 9; // 跳过 "led_id":
            command.led_id = atoi(led_id_start);
            
            if (cmd_end) {
                char cmd_str[32] = {0};
                int cmd_len = cmd_end - cmd_start;
                if (cmd_len < sizeof(cmd_str)) {
                    memcpy(cmd_str, cmd_start, cmd_len);
                    
                    if (strcmp(cmd_str, "LED_ON") == 0) {
                        command.type = CMD_LED_ON;
                    } else if (strcmp(cmd_str, "LED_OFF") == 0) {
                        command.type = CMD_LED_OFF;
                    } else if (strcmp(cmd_str, "LED_TOGGLE") == 0) {
                        command.type = CMD_LED_TOGGLE;
                    } else if (strcmp(cmd_str, "LED_CONTROL") == 0) {
                        // 解析颜色参数
                        char *color_start = strstr(data, "\"color\":\"");
                        char *duration_start = strstr(data, "\"duration\":");
                        
                        if (color_start) {
                            color_start += 9; // 跳过 "color":"
                            char *color_end = strchr(color_start, '"');
                            if (color_end) {
                                char color_str[16] = {0};
                                int color_len = color_end - color_start;
                                if (color_len < sizeof(color_str)) {
                                    memcpy(color_str, color_start, color_len);
                                    led_color_t color = led_color_from_string(color_str);
                                    
                                    if (duration_start) {
                                        // 带时长的颜色控制
                                        command.type = CMD_LED_COLOR_WITH_TIME;
                                        command.param1 = color;
                                        uint32_t duration_ms = atoi(duration_start + 11);
                                        command.param2 = duration_ms & 0xFFFF;
                                        command.param3 = (duration_ms >> 16) & 0xFFFF;
                                    } else {
                                        // 简单颜色控制
                                        command.type = CMD_LED_COLOR;
                                        command.param1 = color;
                                    }
                                } else {
                                    // 没有颜色参数，默认白色
                                    command.type = CMD_LED_COLOR;
                                    command.param1 = LED_COLOR_WHITE;
                                }
                            }
                        }
                    } else if (strcmp(cmd_str, "GET_STATUS") == 0) {
                        mqtt_send_status();
                        return;
                    }
                    
                    // MQTT命令处理 - 直接调用LED控制函数而不是队列
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
                            ESP_LOGW(TAG, "Unknown MQTT command type: %d", command.type);
                            break;
                    }
                }
            }
        }
    }
}

bool mqtt_is_connected(void)
{
    return mqtt_connected;
}

bool wifi_is_connected(void)
{
    return wifi_connected;
}

esp_err_t mqtt_disconnect(void)
{
    if (mqtt_client) {
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }
    mqtt_connected = false;
    g_system_status.mqtt_connected = false;
    
    ESP_LOGI(TAG, "MQTT disconnected");
    return ESP_OK;
}

esp_err_t wifi_disconnect(void)
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    wifi_connected = false;
    g_system_status.wifi_connected = false;
    
    ESP_LOGI(TAG, "WiFi disconnected");
    return ESP_OK;
}

esp_err_t mqtt_manager_reconnect(void)
{
    if (!wifi_connected) {
        ESP_LOGW(TAG, "WiFi not connected, attempting WiFi connection first");
        return wifi_connect();
    }
    
    if (!mqtt_connected) {
        ESP_LOGI(TAG, "Attempting MQTT reconnection");
        return mqtt_connect();
    }
    
    ESP_LOGI(TAG, "MQTT already connected");
    return ESP_OK;
} 