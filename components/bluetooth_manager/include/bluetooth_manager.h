#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include "common.h"
#include "gap.h"
#include "gatt_svc.h"

/* 蓝牙管理器错误代码 */
typedef enum {
    BT_MGR_OK = 0,
    BT_MGR_ERR_INIT_FAILED = -1,
    BT_MGR_ERR_GAP_INIT_FAILED = -2,
    BT_MGR_ERR_GATT_INIT_FAILED = -3,
    BT_MGR_ERR_NOT_INITIALIZED = -4
} bt_mgr_err_t;

/* 蓝牙管理器状态 */
typedef enum {
    BT_MGR_STATE_UNINITIALIZED = 0,
    BT_MGR_STATE_INITIALIZED,
    BT_MGR_STATE_ADVERTISING,
    BT_MGR_STATE_CONNECTED
} bt_mgr_state_t;

/* 公共函数声明 */
bt_mgr_err_t bluetooth_manager_init(void);
bt_mgr_err_t bluetooth_manager_start(void);
bt_mgr_err_t bluetooth_manager_stop(void);
bt_mgr_state_t bluetooth_manager_get_state(void);
bool bluetooth_manager_is_connected(void);

/* 发送通知函数 */
void bluetooth_manager_notify_led_status(void);
void bluetooth_manager_send_response(const char* status, const char* message);

#endif // BLUETOOTH_MANAGER_H 