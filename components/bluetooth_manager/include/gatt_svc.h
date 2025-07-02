#ifndef BT_GATT_SVC_H
#define BT_GATT_SVC_H

/* Includes */
#include "common.h"

/* NimBLE GATT APIs */
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

/* NimBLE GAP APIs */
#include "host/ble_gap.h"

/* LED控制服务UUID (16位) */
#define LED_STRIP_SERVICE_UUID_16           0x00FF

/* LED控制特征值UUID (16位) */
#define LED_STRIP_CMD_CHAR_UUID_16          0xFF01
#define LED_STRIP_NOTIFY_CHAR_UUID_16       0xFF02

/* Public function declarations */
void send_led_status_notification(void);
void send_led_response_notification(const uint8_t* response, uint16_t length);
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
void gatt_svr_subscribe_cb(struct ble_gap_event *event);
int gatt_svc_init(void);

/* 灯条协议处理函数 */
uint8_t parse_led_strip_command(const uint8_t *frame_data, uint16_t frame_len, ble_led_cmd_t *cmd);
void handle_led_strip_command(const ble_led_cmd_t *cmd);
uint16_t create_led_status_frame(uint8_t *frame_buffer, uint16_t buffer_size);
uint16_t create_response_frame(uint8_t cmd_type, uint16_t device_id, const uint8_t *data, uint8_t data_len, uint8_t *frame_buffer, uint16_t buffer_size);

/* 16位和校验函数 */
uint16_t checksum16_calculate(const uint8_t *data, uint16_t length);

/* 协议帧封装和解析函数 */
uint16_t protocol_frame_pack(uint8_t *frame_buf, const uint8_t *data, uint8_t data_len);
uint8_t protocol_frame_parse(const uint8_t *frame_buf, uint16_t frame_len, uint8_t *data, uint8_t *data_len);

#endif // BT_GATT_SVC_H 