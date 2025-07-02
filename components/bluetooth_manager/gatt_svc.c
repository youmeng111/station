#include "gatt_svc.h"

/* Private function declarations */
static int led_strip_cmd_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg);
static int led_strip_notify_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Private variables */
/* LED控制服务 */
static const ble_uuid16_t led_strip_svc_uuid = 
    BLE_UUID16_INIT(LED_STRIP_SERVICE_UUID_16);

/* LED命令特征值 (写入) */
static uint16_t led_strip_cmd_chr_val_handle;
static const ble_uuid16_t led_strip_cmd_chr_uuid = 
    BLE_UUID16_INIT(LED_STRIP_CMD_CHAR_UUID_16);

/* LED响应特征值 (通知) */
static uint16_t led_strip_notify_chr_val_handle;
static const ble_uuid16_t led_strip_notify_chr_uuid = 
    BLE_UUID16_INIT(LED_STRIP_NOTIFY_CHAR_UUID_16);

/* 连接状态 */
static uint16_t current_conn_handle = 0;
static bool conn_handle_inited = false;
static bool notify_enabled = false;

/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** LED Strip Control Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &led_strip_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /*** LED Strip Command Characteristic ***/
                .uuid = &led_strip_cmd_chr_uuid.u,
                .access_cb = led_strip_cmd_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &led_strip_cmd_chr_val_handle,
            },
            {
                /*** LED Strip Notify Characteristic ***/
                .uuid = &led_strip_notify_chr_uuid.u,
                .access_cb = led_strip_notify_chr_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &led_strip_notify_chr_val_handle,
            },
            {
                0, /* No more characteristics in this service */
            },
        }
    },
    {
        0, /* No more services */
    },
};

/* 16位和校验算法实现 */
uint16_t checksum16_calculate(const uint8_t *data, uint16_t length)
{
    uint32_t sum = 0;
    
    // 累加所有数据字节
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    
    // 将进位加回到低16位
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    
    // 返回16位和校验值（取反）
    return ~((uint16_t)sum);
}

/* 协议帧封装函数 */
uint16_t protocol_frame_pack(uint8_t *frame_buf, const uint8_t *data, uint8_t data_len)
{
    uint16_t frame_len = 0;
    uint16_t crc16;
    
    if (!frame_buf || !data || data_len > MAX_PROTOCOL_DATA_LENGTH) {
        return 0;
    }
    
    // 帧头
    frame_buf[frame_len++] = PROTOCOL_FRAME_HEADER_1;
    frame_buf[frame_len++] = PROTOCOL_FRAME_HEADER_2;
    
    // 长度
    frame_buf[frame_len++] = data_len;
    
    // 数据
    memcpy(&frame_buf[frame_len], data, data_len);
    frame_len += data_len;
    
    // 16位和校验(大端格式)
    crc16 = checksum16_calculate(data, data_len);
    frame_buf[frame_len++] = (crc16 >> 8) & 0xFF;  // 校验高字节
    frame_buf[frame_len++] = crc16 & 0xFF;         // 校验低字节
    
    // 帧尾
    frame_buf[frame_len++] = PROTOCOL_FRAME_TAIL_1;
    frame_buf[frame_len++] = PROTOCOL_FRAME_TAIL_2;
    
    return frame_len;
}

/* 协议帧解析函数 */
uint8_t protocol_frame_parse(const uint8_t *frame_buf, uint16_t frame_len, uint8_t *data, uint8_t *data_len)
{
    if (!frame_buf || !data || !data_len || frame_len < 7) {
        return PROTOCOL_ERR_LENGTH;
    }
    
    // 检查帧头
    if (frame_buf[0] != PROTOCOL_FRAME_HEADER_1 || frame_buf[1] != PROTOCOL_FRAME_HEADER_2) {
        return PROTOCOL_ERR_FRAME_HEAD;
    }
    
    // 检查帧尾
    if (frame_buf[frame_len-2] != PROTOCOL_FRAME_TAIL_1 || frame_buf[frame_len-1] != PROTOCOL_FRAME_TAIL_2) {
        return PROTOCOL_ERR_FRAME_TAIL;
    }
    
    // 获取数据长度
    uint8_t expected_data_len = frame_buf[2];
    if (expected_data_len > MAX_PROTOCOL_DATA_LENGTH) {
        return PROTOCOL_ERR_LENGTH;
    }
    
    // 检查帧长度
    uint16_t expected_frame_len = 7 + expected_data_len; // 帧头(2) + 长度(1) + 数据 + 校验(2) + 帧尾(2)
    if (frame_len != expected_frame_len) {
        return PROTOCOL_ERR_LENGTH;
    }
    
    // 提取数据
    memcpy(data, &frame_buf[3], expected_data_len);
    *data_len = expected_data_len;
    
    // 校验和校验
    uint16_t received_checksum = (frame_buf[3 + expected_data_len] << 8) | frame_buf[4 + expected_data_len];
    uint16_t calculated_checksum = checksum16_calculate(data, expected_data_len);
    
    if (received_checksum != calculated_checksum) {
        ESP_LOGE(BT_TAG, "Checksum mismatch: received=0x%04X, calculated=0x%04X", received_checksum, calculated_checksum);
        return PROTOCOL_ERR_CHECKSUM;
    }
    
    return PROTOCOL_OK;
}

/* 灯条协议命令解析 */
uint8_t parse_led_strip_command(const uint8_t *frame_data, uint16_t frame_len, ble_led_cmd_t *cmd)
{
    uint8_t data[MAX_PROTOCOL_DATA_LENGTH];
    uint8_t data_len;
    
    // 解析协议帧
    uint8_t parse_result = protocol_frame_parse(frame_data, frame_len, data, &data_len);
    if (parse_result != PROTOCOL_OK) {
        return parse_result;
    }
    
    if (data_len < 4) { // 至少需要: 命令类型(1) + 数据长度(1) + 设备ID(2)
        return PROTOCOL_ERR_LENGTH;
    }
    
    // 解析命令数据
    station_to_strip_cmd_t *station_cmd = (station_to_strip_cmd_t *)data;
    
    // 清空命令结构
    memset(cmd, 0, sizeof(ble_led_cmd_t));
    
    // 设备ID (转换为小端格式)
    uint16_t device_id = (station_cmd->device_id >> 8) | (station_cmd->device_id << 8);
    
    ESP_LOGI(BT_TAG, "Received command: type=0x%02X, device_id=0x%04X", 
             station_cmd->cmd_type, device_id);
    
    // 根据命令类型填充LED命令结构
    switch (station_cmd->cmd_type) {
    case CMD_SET_LIGHT_DURATION:
        cmd->type = CMD_LED_COLOR_WITH_TIME;
        cmd->led_id = (uint8_t)(device_id & 0xFF);
        cmd->param1 = LED_COLOR_WHITE; // 默认白色
        if (station_cmd->data_length >= 2) {
            // 时长 (大端转小端，从秒转换为毫秒)
            uint16_t duration_sec = (station_cmd->cmd_data[0] << 8) | station_cmd->cmd_data[1];
            uint32_t duration_ms = duration_sec * 1000;
            cmd->param2 = duration_ms; // 存储时长(毫秒)
        } else {
            cmd->param2 = 0; // 永久点亮
        }
        break;
        
    case CMD_SET_LIGHT_COLOR:
        cmd->led_id = (uint8_t)(device_id & 0xFF);
        if (station_cmd->data_length >= 1) {
            // 直接使用颜色枚举值
            if (station_cmd->cmd_data[0] == LED_COLOR_OFF) {
                cmd->type = CMD_LED_OFF;
            } else {
                cmd->type = CMD_LED_COLOR;
                cmd->param1 = station_cmd->cmd_data[0]; // 颜色枚举值
            }
        } else {
            cmd->type = CMD_LED_COLOR;
            cmd->param1 = LED_COLOR_WHITE; // 默认白色
        }
        break;
        
    case CMD_SET_BLINK_MODE:
        // 这里可以根据需要实现闪烁模式，映射为颜色控制
        cmd->type = CMD_LED_COLOR;
        cmd->led_id = (uint8_t)(device_id & 0xFF);
        if (station_cmd->data_length >= 1) {
            // 根据闪烁模式设置颜色，闪烁用白色表示
            switch (station_cmd->cmd_data[0]) {
            case BLINK_MODE_NONE: cmd->param1 = LED_COLOR_WHITE; break; // 常亮白色
            case BLINK_MODE_SLOW: cmd->param1 = LED_COLOR_YELLOW; break; // 慢闪用黄色
            case BLINK_MODE_FAST: cmd->param1 = LED_COLOR_RED;  break; // 快闪用红色
            default: cmd->param1 = LED_COLOR_WHITE; break;
            }
        } else {
            cmd->param1 = LED_COLOR_WHITE; // 默认白色
        }
        break;
        
    case CMD_SET_BEEP_STATE:
    case CMD_SET_CONTROL_MODE:
        // 这些命令在基站不直接处理LED，只记录状态
        cmd->type = CMD_SYSTEM_STATUS;
        cmd->led_id = (uint8_t)(device_id & 0xFF);
        break;
        
    case CMD_GET_BATTERY_STATUS:
        cmd->type = CMD_SYSTEM_STATUS;
        cmd->led_id = (uint8_t)(device_id & 0xFF);
        break;
        
    default:
        ESP_LOGE(BT_TAG, "Unknown LED strip command: 0x%02X", station_cmd->cmd_type);
        return PROTOCOL_ERR_LENGTH;
    }
    
    return PROTOCOL_OK;
}

/* LED命令处理 */
void handle_led_strip_command(const ble_led_cmd_t *cmd)
{
    if (!cmd) {
        return;
    }

    switch (cmd->type) {
    case CMD_LED_ON:
        ESP_LOGI(BT_TAG, "Executing LED ON command for LED %d", cmd->led_id);
        led_controller_turn_on(cmd->led_id);
        break;

    case CMD_LED_OFF:
        ESP_LOGI(BT_TAG, "Executing LED OFF command for LED %d", cmd->led_id);
        led_controller_turn_off(cmd->led_id);
        break;

    case CMD_LED_COLOR:
        ESP_LOGI(BT_TAG, "Executing LED COLOR command for LED %d, color: %d", 
                 cmd->led_id, cmd->param1);
        led_controller_set_color_simple(cmd->led_id, (led_color_t)cmd->param1);
        break;

    case CMD_LED_COLOR_WITH_TIME:
        ESP_LOGI(BT_TAG, "Executing LED COLOR WITH TIME command for LED %d, color: %d, duration: %lu ms", 
                 cmd->led_id, cmd->param1, cmd->param2);
        led_controller_set_color_with_time(cmd->led_id, (led_color_t)cmd->param1, cmd->param2);
        break;

    case CMD_SYSTEM_STATUS:
        ESP_LOGI(BT_TAG, "Executing STATUS command for LED %d", cmd->led_id);
        // 发送状态响应
        send_led_status_notification();
        break;

    default:
        ESP_LOGW(BT_TAG, "Unknown LED command: %d", cmd->type);
        break;
    }
}

/* 创建LED状态帧 */
uint16_t create_led_status_frame(uint8_t *frame_buffer, uint16_t buffer_size)
{
    if (!frame_buffer || buffer_size < MAX_PROTOCOL_FRAME_LENGTH) {
        return 0;
    }

    strip_to_station_data_t response_data;
    memset(&response_data, 0, sizeof(response_data));
    
    // 填充响应数据
    response_data.cmd_type = CMD_GET_BATTERY_STATUS;
    response_data.device_id = 0x0001; // 基站设备ID (大端格式)
    
    // 添加时间戳 (4字节)
    uint32_t timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
    response_data.data[0] = (timestamp >> 24) & 0xFF;
    response_data.data[1] = (timestamp >> 16) & 0xFF;
    response_data.data[2] = (timestamp >> 8) & 0xFF;
    response_data.data[3] = timestamp & 0xFF;
    
    // 添加LED状态 (最多4个LED，每个LED 3字节：ID + 开关 + 颜色)
    uint8_t status_idx = 4;
    for (int i = 1; i <= 4 && status_idx <= 12; i++) {
        response_data.data[status_idx++] = i; // LED ID
        response_data.data[status_idx++] = led_controller_is_on(i) ? 1 : 0; // 开关状态
        response_data.data[status_idx++] = (uint8_t)led_controller_get_color(i); // 颜色
    }
    
    return protocol_frame_pack(frame_buffer, (uint8_t*)&response_data, sizeof(response_data));
}

/* 创建响应帧 */
uint16_t create_response_frame(uint8_t cmd_type, uint16_t device_id, const uint8_t *data, uint8_t data_len, uint8_t *frame_buffer, uint16_t buffer_size)
{
    if (!frame_buffer || buffer_size < MAX_PROTOCOL_FRAME_LENGTH) {
        return 0;
    }

    strip_to_station_data_t response_data;
    memset(&response_data, 0, sizeof(response_data));
    
    response_data.cmd_type = cmd_type;
    response_data.device_id = device_id; // 保持原始字节序
    
    if (data && data_len > 0) {
        uint8_t copy_len = (data_len > sizeof(response_data.data)) ? sizeof(response_data.data) : data_len;
        memcpy(response_data.data, data, copy_len);
    }
    
    return protocol_frame_pack(frame_buffer, (uint8_t*)&response_data, sizeof(response_data));
}

static int led_strip_cmd_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        ESP_LOGI(BT_TAG, "LED strip command write; conn_handle=%d attr_handle=%d length=%d",
                 conn_handle, attr_handle, ctxt->om->om_len);

        if (attr_handle == led_strip_cmd_chr_val_handle) {
            if (ctxt->om->om_len >= 7 && ctxt->om->om_len <= MAX_PROTOCOL_FRAME_LENGTH) {
                /* 复制命令数据 */
                uint8_t frame_buf[MAX_PROTOCOL_FRAME_LENGTH];
                memset(frame_buf, 0, sizeof(frame_buf));
                rc = ble_hs_mbuf_to_flat(ctxt->om, frame_buf, ctxt->om->om_len, NULL);
                if (rc == 0) {
                    ESP_LOGI(BT_TAG, "Received LED strip frame: %d bytes", ctxt->om->om_len);
                    
                    /* 解析LED strip命令 */
                    ble_led_cmd_t led_cmd;
                    uint8_t parse_result = parse_led_strip_command(frame_buf, ctxt->om->om_len, &led_cmd);
                    
                    if (parse_result == PROTOCOL_OK) {
                        /* 处理LED命令 */
                        handle_led_strip_command(&led_cmd);
                        
                        /* 发送简化的成功响应（适配20字节MTU限制） */
                        uint8_t simple_response[20];
                        memset(simple_response, 0, sizeof(simple_response));
                        
                        // 简化协议格式：AA 55 [长度] [命令类型] [状态] [LED_ID] [颜色] ... [余位补0]
                        simple_response[0] = 0xAA;  // 帧头1
                        simple_response[1] = 0x55;  // 帧头2
                        simple_response[2] = 0x10;  // 数据长度16字节（适配20字节总长度）
                        simple_response[3] = 0xFF;  // 响应命令类型
                        simple_response[4] = 0x00;  // 成功状态
                        simple_response[5] = led_cmd.led_id;  // LED ID
                        simple_response[6] = led_cmd.param1;  // 颜色
                        simple_response[7] = led_controller_is_on(led_cmd.led_id) ? 1 : 0;  // 开关状态
                        // 其余字节保持0
                        
                        send_led_response_notification(simple_response, 20);
                    } else {
                        /* 发送简化的错误响应（适配20字节MTU限制） */
                        uint8_t simple_response[20];
                        memset(simple_response, 0, sizeof(simple_response));
                        
                        simple_response[0] = 0xAA;  // 帧头1
                        simple_response[1] = 0x55;  // 帧头2
                        simple_response[2] = 0x10;  // 数据长度16字节
                        simple_response[3] = 0xFF;  // 响应命令类型
                        simple_response[4] = parse_result;  // 错误状态
                        // 其余字节保持0
                        
                        send_led_response_notification(simple_response, 20);
                    }
                }
                return 0;
            } else {
                ESP_LOGE(BT_TAG, "Invalid frame length: %d bytes", ctxt->om->om_len);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
        }
        return BLE_ATT_ERR_UNLIKELY;

    default:
        ESP_LOGE(BT_TAG, "Unexpected access operation to LED strip command characteristic: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int led_strip_notify_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg) {
    /* 响应特征值只支持通知，不支持读写 */
    ESP_LOGE(BT_TAG, "Unexpected access operation to LED strip notify characteristic: %d", ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}

/* Public functions */
void send_led_status_notification(void) {
    if (!conn_handle_inited || !notify_enabled) {
        ESP_LOGW(BT_TAG, "Cannot send status notification: conn_handle_inited=%d, notify_enabled=%d",
                 conn_handle_inited, notify_enabled);
        return;
    }

    uint8_t status_buffer[MAX_PROTOCOL_FRAME_LENGTH];
    uint16_t status_len = create_led_status_frame(status_buffer, sizeof(status_buffer));
    
    if (status_len > 0) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(status_buffer, status_len);
        if (om != NULL) {
            int rc = ble_gattc_notify_custom(current_conn_handle, led_strip_notify_chr_val_handle, om);
            if (rc != 0) {
                ESP_LOGE(BT_TAG, "Failed to send status notification: %d", rc);
            } else {
                ESP_LOGI(BT_TAG, "Status notification sent successfully");
            }
        } else {
            ESP_LOGE(BT_TAG, "Failed to create mbuf for status notification");
        }
    }
}

void send_led_response_notification(const uint8_t* response, uint16_t length) {
    if (!conn_handle_inited || !notify_enabled) {
        ESP_LOGW(BT_TAG, "Cannot send response notification: conn_handle_inited=%d, notify_enabled=%d",
                 conn_handle_inited, notify_enabled);
        return;
    }

    if (!response || length == 0) {
        ESP_LOGE(BT_TAG, "Invalid response data");
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(response, length);
    if (om != NULL) {
        int rc = ble_gattc_notify_custom(current_conn_handle, led_strip_notify_chr_val_handle, om);
        if (rc != 0) {
            ESP_LOGE(BT_TAG, "Failed to send response notification: %d", rc);
        } else {
            ESP_LOGI(BT_TAG, "Response notification sent successfully (%d bytes)", length);
            /* 打印发送的数据内容（前8字节用于调试） */
            if (length >= 8) {
                ESP_LOGI(BT_TAG, "Response data: %02X %02X %02X %02X %02X %02X %02X %02X...", 
                         response[0], response[1], response[2], response[3], 
                         response[4], response[5], response[6], response[7]);
            }
        }
    } else {
        ESP_LOGE(BT_TAG, "Failed to create mbuf for response notification");
    }
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(BT_TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(BT_TAG, "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle,
                 ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(BT_TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

void gatt_svr_subscribe_cb(struct ble_gap_event *event) {
    struct ble_gap_conn_desc desc;
    int rc;

    if (event->subscribe.attr_handle == led_strip_notify_chr_val_handle) {
        if (event->subscribe.cur_notify) {
            ESP_LOGI(BT_TAG, "LED strip notifications enabled");
            notify_enabled = true;
            
            /* 协商更大的MTU以支持完整的协议帧 */
            rc = ble_gattc_exchange_mtu(event->subscribe.conn_handle, NULL, NULL);
            if (rc != 0) {
                ESP_LOGW(BT_TAG, "MTU exchange failed: %d", rc);
            } else {
                ESP_LOGI(BT_TAG, "MTU exchange initiated");
            }
        } else {
            ESP_LOGI(BT_TAG, "LED strip notifications disabled");
            notify_enabled = false;
        }
        
        current_conn_handle = event->subscribe.conn_handle;
        conn_handle_inited = true;
        
        rc = ble_gap_conn_find(current_conn_handle, &desc);
        if (rc == 0) {
            ESP_LOGI(BT_TAG, "Connection established with " ADDR_FORMAT,
                     ADDR_FORMAT_ARGS(desc.peer_id_addr.val));
        }
    }
}

int gatt_svc_init(void) {
    int rc;

    /* 只初始化GATT服务，GAP服务由蓝牙管理器初始化 */
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
} 