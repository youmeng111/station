#include "mqtt_manager.h"
#include "system_config.h"
#include "led_controller.h"
#include "esp_log.h"
#include "esp_event.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

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
    
    return mqtt_publish(MQTT_TOPIC_STATUS, info_buffer, 1, 1);
}

// 发送状态响应
static void mqtt_send_response(const char *command_type, const char *status, const char *device_id, const char *error_msg)
{
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "response_to", command_type);
    cJSON_AddStringToObject(response, "status", status);
    if (device_id) {
        cJSON_AddStringToObject(response, "device_id", device_id);
    }
    if (error_msg) {
        cJSON_AddStringToObject(response, "error", error_msg);
    }
    
    char *response_str = cJSON_Print(response);
    if (response_str) {
        mqtt_publish(MQTT_TOPIC_STATUS, response_str, 1, 0);
        free(response_str);
    }
    cJSON_Delete(response);
}

// 发送异常消息
static void mqtt_send_exception(const char *exception_type, const char *device_id, const char *message)
{
    cJSON *exception = cJSON_CreateObject();
    cJSON_AddStringToObject(exception, "exception_type", exception_type);
    cJSON_AddStringToObject(exception, "timestamp", "");  // 可以添加时间戳
    
    cJSON *parameters = cJSON_CreateObject();
    if (device_id) {
        cJSON_AddStringToObject(parameters, "device_id", device_id);
    }
    if (message) {
        cJSON_AddStringToObject(parameters, "message", message);
    }
    cJSON_AddItemToObject(exception, "parameters", parameters);
    
    char *exception_str = cJSON_Print(exception);
    if (exception_str) {
        mqtt_publish(MQTT_TOPIC_EXCEPTION, exception_str, 1, 0);
        free(exception_str);
    }
    cJSON_Delete(exception);
}

void mqtt_handle_message(const char *topic, const char *data, int data_len)
{
    ESP_LOGI(TAG, "Handling MQTT message on topic: %s", topic);
    
    // 只处理命令主题
    if (strcmp(topic, MQTT_TOPIC_COMMAND) != 0) {
        return;
    }
    
    // 解析JSON命令
    cJSON *json = cJSON_ParseWithLength(data, data_len);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON command");
        mqtt_send_exception("JSON_PARSE_ERROR", NULL, "Invalid JSON format");
        return;
    }
    
    cJSON *command_type = cJSON_GetObjectItem(json, "command_type");
    if (!cJSON_IsString(command_type)) {
        ESP_LOGE(TAG, "Missing or invalid command_type field");
        cJSON_Delete(json);
        mqtt_send_exception("INVALID_COMMAND", NULL, "Missing command_type field");
        return;
    }
    
    cJSON *parameters = cJSON_GetObjectItem(json, "parameters");
    if (!cJSON_IsObject(parameters)) {
        ESP_LOGE(TAG, "Missing or invalid parameters field");
        cJSON_Delete(json);
        mqtt_send_exception("INVALID_COMMAND", NULL, "Missing parameters field");
        return;
    }
    
    const char *cmd = command_type->valuestring;
    ESP_LOGI(TAG, "Processing command: %s", cmd);
        
    // 解析device_id参数
    cJSON *device_id_json = cJSON_GetObjectItem(parameters, "device_id");
    const char *device_id = cJSON_IsString(device_id_json) ? device_id_json->valuestring : NULL;
    uint8_t led_id = device_id ? (uint8_t)atoi(device_id) : 0;
    
    /* ========== 协议规范命令处理 ========== */
    
    if (strcmp(cmd, "SET_LIGHT_COLOR") == 0) {
        cJSON *color_json = cJSON_GetObjectItem(parameters, "color");
        if (cJSON_IsString(color_json)) {
            led_color_t color = led_color_from_string(color_json->valuestring);
            led_strip_set_color(led_id, color);
            mqtt_send_response(cmd, "success", device_id, NULL);
            ESP_LOGI(TAG, "Set light color for device %s to %s", device_id, color_json->valuestring);
        } else {
            mqtt_send_response(cmd, "error", device_id, "Invalid color parameter");
        }
    }
    
    else if (strcmp(cmd, "SET_LIGHT_DURATION") == 0) {
        cJSON *duration_json = cJSON_GetObjectItem(parameters, "duration");
        cJSON *color_json = cJSON_GetObjectItem(parameters, "color");
        
        if (cJSON_IsNumber(duration_json)) {
            uint32_t duration_sec = (uint32_t)duration_json->valueint;
            uint32_t duration_ms = duration_sec * 1000;  // 转换为毫秒
            
            led_color_t color = LED_COLOR_WHITE;  // 默认白色
            if (cJSON_IsString(color_json)) {
                color = led_color_from_string(color_json->valuestring);
            }
            
            led_strip_set_color_with_duration(led_id, color, duration_ms);
            mqtt_send_response(cmd, "success", device_id, NULL);
            ESP_LOGI(TAG, "Set light duration for device %s: %" PRIu32 " seconds", device_id, duration_sec);
        } else {
            mqtt_send_response(cmd, "error", device_id, "Invalid duration parameter");
        }
    }
    
    else if (strcmp(cmd, "SET_BLINK_MODE") == 0) {
        cJSON *mode_json = cJSON_GetObjectItem(parameters, "mode");
        if (cJSON_IsString(mode_json)) {
            const char *mode = mode_json->valuestring;
            
            // 简化实现：用颜色变化模拟闪烁
            if (strcmp(mode, "NO_BLINK") == 0) {
                led_strip_set_color(led_id, LED_COLOR_WHITE);
            } else if (strcmp(mode, "SLOW") == 0) {
                // 慢闪：白色5秒，关闭1秒，循环
                led_strip_set_color_with_duration(led_id, LED_COLOR_WHITE, 5000);
            } else if (strcmp(mode, "FAST") == 0) {
                // 快闪：白色1秒，关闭0.5秒，循环
                led_strip_set_color_with_duration(led_id, LED_COLOR_WHITE, 1000);
            }
            
            mqtt_send_response(cmd, "success", device_id, NULL);
            ESP_LOGI(TAG, "Set blink mode for device %s: %s", device_id, mode);
        } else {
            mqtt_send_response(cmd, "error", device_id, "Invalid blink mode");
        }
    }
    
    else if (strcmp(cmd, "SET_BEEP_STATE") == 0) {
        cJSON *state_json = cJSON_GetObjectItem(parameters, "state");
        if (cJSON_IsString(state_json)) {
            const char *state = state_json->valuestring;
            
            // 蜂鸣器功能暂未实现，返回成功状态
            mqtt_send_response(cmd, "success", device_id, NULL);
            ESP_LOGI(TAG, "Set beep state for device %s: %s (Not implemented)", device_id, state);
        } else {
            mqtt_send_response(cmd, "error", device_id, "Invalid beep state");
                        }
    }
    
    else if (strcmp(cmd, "SET_CONTROL_MODE") == 0) {
        cJSON *mode_json = cJSON_GetObjectItem(parameters, "mode");
        if (cJSON_IsString(mode_json)) {
            const char *mode = mode_json->valuestring;
            
            // 控制模式功能暂存储在系统配置中
            mqtt_send_response(cmd, "success", device_id, NULL);
            ESP_LOGI(TAG, "Set control mode for device %s: %s", device_id, mode);
        } else {
            mqtt_send_response(cmd, "error", device_id, "Invalid control mode");
        }
    }
    
    else if (strcmp(cmd, "GET_BATTERY_STATUS") == 0) {
        // 构造电池状态响应
        cJSON *status_response = cJSON_CreateObject();
        cJSON_AddStringToObject(status_response, "status_type", "BATTERY_STATUS");
        
        cJSON *status_params = cJSON_CreateObject();
        cJSON_AddStringToObject(status_params, "device_id", device_id ? device_id : "0001");
        cJSON_AddStringToObject(status_params, "battery_level", "HIGH");  // 模拟数据
        cJSON_AddNumberToObject(status_params, "battery_percentage", 85);
        cJSON_AddNumberToObject(status_params, "battery_voltage", 3700);
        cJSON_AddItemToObject(status_response, "parameters", status_params);
        
        char *status_str = cJSON_Print(status_response);
        if (status_str) {
            mqtt_publish(MQTT_TOPIC_STATUS, status_str, 1, 0);
            free(status_str);
                    }
        cJSON_Delete(status_response);
        
        ESP_LOGI(TAG, "Sent battery status for device %s", device_id);
    }
    
    else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd);
        mqtt_send_response(cmd, "error", device_id, "Unknown command type");
                    }
    
    cJSON_Delete(json);
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