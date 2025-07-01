#include "bluetooth_manager.h"

/* Library function declarations */
void ble_store_config_init(void);

/* Private function declarations */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_config_init(void);
static void nimble_host_task(void *param);

/* Private variables */
static bt_mgr_state_t bt_manager_state = BT_MGR_STATE_UNINITIALIZED;

/* Private functions */
static void on_stack_reset(int reason) {
    ESP_LOGI(BT_TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    ESP_LOGI(BT_TAG, "nimble stack synced");
    /* 栈同步后开始广播 */
    adv_init();
    bt_manager_state = BT_MGR_STATE_ADVERTISING;
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param) {
    ESP_LOGI(BT_TAG, "nimble host task started");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

/* Public functions */
bt_mgr_err_t bluetooth_manager_init(void) {
    esp_err_t ret;
    int rc;

    if (bt_manager_state != BT_MGR_STATE_UNINITIALIZED) {
        ESP_LOGW(BT_TAG, "Bluetooth manager already initialized");
        return BT_MGR_OK;
    }

    ESP_LOGI(BT_TAG, "Initializing Bluetooth manager...");

    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(BT_TAG, "Failed to initialize nimble stack: %d", ret);
        return BT_MGR_ERR_INIT_FAILED;
    }

    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "Failed to initialize GAP service: %d", rc);
        return BT_MGR_ERR_GAP_INIT_FAILED;
    }

    /* GATT server initialization */
    rc = gatt_svc_init();
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "Failed to initialize GATT server: %d", rc);
        return BT_MGR_ERR_GATT_INIT_FAILED;
    }

    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    bt_manager_state = BT_MGR_STATE_INITIALIZED;
    ESP_LOGI(BT_TAG, "Bluetooth manager initialized successfully");

    return BT_MGR_OK;
}

bt_mgr_err_t bluetooth_manager_start(void) {
    if (bt_manager_state == BT_MGR_STATE_UNINITIALIZED) {
        ESP_LOGE(BT_TAG, "Bluetooth manager not initialized");
        return BT_MGR_ERR_NOT_INITIALIZED;
    }

    if (bt_manager_state != BT_MGR_STATE_INITIALIZED) {
        ESP_LOGW(BT_TAG, "Bluetooth manager already started");
        return BT_MGR_OK;
    }

    ESP_LOGI(BT_TAG, "Starting Bluetooth manager...");

    /* Start NimBLE host task thread */
    xTaskCreate(nimble_host_task, "NimBLE Host", 4*1024, NULL, 5, NULL);

    ESP_LOGI(BT_TAG, "Bluetooth manager started");
    return BT_MGR_OK;
    }

bt_mgr_err_t bluetooth_manager_stop(void) {
    if (bt_manager_state == BT_MGR_STATE_UNINITIALIZED) {
        return BT_MGR_ERR_NOT_INITIALIZED;
    }

    ESP_LOGI(BT_TAG, "Stopping Bluetooth manager...");

    /* 停止NimBLE栈 */
    int rc = nimble_port_stop();
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "Failed to stop nimble port: %d", rc);
    }

    /* 反初始化NimBLE栈 */
    nimble_port_deinit();

    bt_manager_state = BT_MGR_STATE_UNINITIALIZED;
    ESP_LOGI(BT_TAG, "Bluetooth manager stopped");

    return BT_MGR_OK;
    }

bt_mgr_state_t bluetooth_manager_get_state(void) {
    return bt_manager_state;
}

bool bluetooth_manager_is_connected(void) {
    return bt_manager_state == BT_MGR_STATE_CONNECTED;
}

void bluetooth_manager_notify_led_status(void) {
    send_led_status_notification();
}

void bluetooth_manager_send_response(const char* status, const char* message) {
    uint8_t response_frame[MAX_PROTOCOL_FRAME_LENGTH];
    uint8_t response_data[16];
    uint8_t data_len = 0;
    
    // 构造响应数据
    if (message) {
        data_len = strlen(message);
        if (data_len > sizeof(response_data)) {
            data_len = sizeof(response_data);
        }
        memcpy(response_data, message, data_len);
    }
    
    // 创建响应帧
    uint16_t response_len = create_response_frame(0xFF, 0x0001, response_data, data_len, response_frame, sizeof(response_frame));
    if (response_len > 0) {
        send_led_response_notification(response_frame, response_len);
    }
} 