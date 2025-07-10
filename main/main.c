/*
 * ESP32快递驿站智能标签基站主程序 - BLE广播控制版本
 * 支持大规模设备控制（5000台+）
 * 应用场景：快递驿站包裹管理系统
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"

/* 组件头文件 */
#include "system_config.h"
#include "led_controller.h"
#include "bluetooth_manager.h"
#include "mqtt_manager.h"
#include "ota_manager.h"

static const char *TAG = "EXPRESS_STATION";

/* 广播控制演示任务 */
static void broadcast_demo_task(void *param);
static void status_monitor_task(void *param);

/* 广播发送完成回调函数 */
static void on_broadcast_sent(bt_mgr_err_t result, uint16_t sequence) {
    if (result == BT_MGR_OK) {
        ESP_LOGI(TAG, "[BROADCAST_OK] Command sequence %d sent successfully! (^_^)", sequence);
    } else {
        ESP_LOGW(TAG, "[BROADCAST_ERR] Command sequence %d failed: %d (x_x)", sequence, result);
    }
}

/* 设备反馈回调函数 (预留，如果后续设备支持反馈) */
static void on_device_feedback(uint32_t tag_id, uint8_t response_type, const uint8_t *data, uint8_t length) {
    ESP_LOGI(TAG, "[FEEDBACK] Received feedback from tag %lu", tag_id);
    ESP_LOGI(TAG, "[FEEDBACK] Type: %d, Length: %d", response_type, length);
    if (length > 0) {
        ESP_LOG_BUFFER_HEX(TAG, data, length);
    }
}

/* 广播控制演示任务 */
static void broadcast_demo_task(void *param) {
    ESP_LOGI(TAG, "[DEMO] Starting broadcast demo task (>_<)");
    
    // 等待蓝牙管理器准备就绪
    while (bluetooth_manager_get_state() != BT_MGR_STATE_READY) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ESP_LOGI(TAG, "[DEMO] Bluetooth manager ready, starting demo sequence...");
    
    uint32_t demo_step = 0;
    
    while (1) {
        demo_step++;
        ESP_LOGI(TAG, "\n===== BROADCAST DEMO STEP %lu =====", demo_step);
        
        switch (demo_step % 8) {
            case 1:
                // 演示1：全部设备亮绿灯
                ESP_LOGI(TAG, "[DEMO1] Broadcasting GREEN to ALL devices");
                bluetooth_manager_broadcast_all_set_color(LED_COLOR_GREEN, BLINK_MODE_NONE, 5000);
                break;
                
            case 2:
                // 演示2：货架A区域设备亮红灯闪烁（过期包裹）
                ESP_LOGI(TAG, "[DEMO2] Broadcasting RED BLINK to Shelf A (expired packages)");
                bluetooth_manager_broadcast_group_set_color(GROUP_SHELF_A, LED_COLOR_RED, BLINK_MODE_FAST, 3000);
                break;
                
            case 3:
                // 演示3：取件台设备亮蓝灯（待取件）
                ESP_LOGI(TAG, "[DEMO3] Broadcasting BLUE to Counter Area (pending pickup)");
                bluetooth_manager_broadcast_group_set_color(GROUP_COUNTER, LED_COLOR_BLUE, BLINK_MODE_SLOW, 4000);
                break;
                
            case 4:
                // 演示4：全部设备蜂鸣器单响
                ESP_LOGI(TAG, "[DEMO4] Broadcasting BEEP to ALL devices");
                bluetooth_manager_broadcast_beep(GROUP_ALL, BEEP_MODE_SINGLE);
                break;
                
            case 5:
                // 演示5：批量激活特定标签（模拟包裹取件）
                ESP_LOGI(TAG, "[DEMO5] Activating multiple tags (package pickup simulation)");
                uint32_t package_ids[] = {1001, 1002, 1003, 1004};
                bluetooth_manager_activate_multiple_tags(package_ids, 4, LED_COLOR_YELLOW, true);
                break;
                
            case 6:
                // 演示6：电池预警广播
                ESP_LOGI(TAG, "[DEMO6] BATTERY WARNING simulation");
                bluetooth_manager_broadcast_battery_warning(15, 2700, 1); // 15%电量，2.7V，预警类型1
                break;
                
            case 7:
                // 演示7：包裹取件提醒（特殊包裹区域）
                ESP_LOGI(TAG, "[DEMO7] Package pickup alert for Special Area");
                bluetooth_manager_package_pickup_alert(GROUP_SPECIAL, LED_COLOR_PURPLE);
                
                // 额外演示：盘点和健康检查
                vTaskDelay(pdMS_TO_TICKS(2000));
                ESP_LOGI(TAG, "[DEMO7+] Starting inventory scan");
                bluetooth_manager_start_inventory_scan(12345, 0, GROUP_ALL, 10000); // 全量盘点
                
                vTaskDelay(pdMS_TO_TICKS(3000));
                ESP_LOGI(TAG, "[DEMO7+] Starting health check");
                bluetooth_manager_start_health_check(HEALTH_CHECK_ALL, 54321, GROUP_ALL);
                break;
                
            case 0:
                // 演示8：快递驿站模式
                ESP_LOGI(TAG, "[DEMO8] Express station mode activation");
                bluetooth_manager_express_station_mode();
                
                // 5秒后关闭所有设备
                vTaskDelay(pdMS_TO_TICKS(5000));
                ESP_LOGI(TAG, "[DEMO8] Turning off all devices");
                bluetooth_manager_broadcast_all_off();
                break;
        }
        
        // 显示广播统计
        const broadcast_stats_t *stats = bluetooth_manager_get_stats();
        ESP_LOGI(TAG, "[STATS] Total: %lu, Success: %lu, Failed: %lu", 
                 stats->total_broadcasts, stats->successful_broadcasts, stats->failed_broadcasts);
        
        // 等待下一个演示步骤
        vTaskDelay(pdMS_TO_TICKS(8000)); // 8秒间隔
    }
}

/* 系统状态监控任务 */
static void status_monitor_task(void *param) {
    ESP_LOGI(TAG, "[MONITOR] Starting system status monitor (^_^)");
    
    while (1) {
        // 获取系统状态
        ESP_LOGI(TAG, "\n===== SYSTEM STATUS =====");
        ESP_LOGI(TAG, "[SYS] Free heap: %lu bytes", esp_get_free_heap_size());
        ESP_LOGI(TAG, "[SYS] Uptime: %llu seconds", esp_timer_get_time() / 1000000);
        
        // 获取蓝牙管理器状态
        bt_mgr_state_t bt_state = bluetooth_manager_get_state();
        const char* state_names[] = {"UNINITIALIZED", "INITIALIZED", "READY", "BROADCASTING", "ERROR"};
        ESP_LOGI(TAG, "[BT] Manager state: %s", state_names[bt_state]);
        
        // 获取广播统计
        const broadcast_stats_t *stats = bluetooth_manager_get_stats();
        ESP_LOGI(TAG, "[BT] Broadcast stats:");
        ESP_LOGI(TAG, "[BT]   Total: %lu", stats->total_broadcasts);
        ESP_LOGI(TAG, "[BT]   Success: %lu", stats->successful_broadcasts);
        ESP_LOGI(TAG, "[BT]   Failed: %lu", stats->failed_broadcasts);
        ESP_LOGI(TAG, "[BT]   Current Sequence: %d", stats->current_sequence);
        ESP_LOGI(TAG, "[BT]   Last Broadcast: %lld ms ago", 
                (esp_timer_get_time() / 1000) - stats->last_broadcast_time);
        
        // 获取设备分组信息
        device_group_t groups[MAX_DEVICE_GROUPS];
        int group_count = bluetooth_manager_get_all_groups(groups, MAX_DEVICE_GROUPS);
        ESP_LOGI(TAG, "[GROUPS] Active groups: %d", group_count);
        for (int i = 0; i < group_count && i < 5; i++) { // 只显示前5个分组
            ESP_LOGI(TAG, "[GROUPS]   %d: %s (%lu devices, %s)", 
                     groups[i].group_id, groups[i].group_name, 
                     groups[i].estimated_device_count,
                     groups[i].is_active ? "ACTIVE" : "INACTIVE");
        }
        
        // 检查系统健康状态
        if (esp_get_free_heap_size() < 50000) {
            ESP_LOGW(TAG, "[WARNING] Low memory detected!");
        }
        
        // 每30秒监控一次
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

/* 便利测试函数 */
void quick_test_commands(void) {
    ESP_LOGI(TAG, "[QUICK_TEST] Starting quick test commands...");
    
    // 等待系统准备
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "[TEST] All devices RED");
    BT_BROADCAST_RED_ALL();
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "[TEST] All devices GREEN");
    BT_BROADCAST_GREEN_ALL();
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "[TEST] All devices BLUE");
    BT_BROADCAST_BLUE_ALL();
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "[TEST] All devices BLINK RED");
    BT_BROADCAST_BLINK_RED_ALL();
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    ESP_LOGI(TAG, "[TEST] All devices OFF");
    BT_BROADCAST_OFF_ALL();
    
    ESP_LOGI(TAG, "[QUICK_TEST] Test completed!");
}

void app_main(void) {
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32 Express Station - BLE Broadcast");
    ESP_LOGI(TAG, "    Version: 2.0.0 (No Checksum)");
    ESP_LOGI(TAG, "    Support: 5000+ smart tags");
    ESP_LOGI(TAG, "    Scenario: Express station management");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "\n");
    
    /* 初始化NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "[INIT] NVS flash initialized (^_^)");

    /* 初始化系统配置 */
    system_config_init();
    ESP_LOGI(TAG, "[INIT] System configuration loaded (^_^)");

    /* 初始化LED控制器 */
    QueueHandle_t led_cmd_queue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(led_command_t));
    if (!led_cmd_queue) {
        ESP_LOGE(TAG, "[ERROR] Failed to create LED command queue (x_x)");
        return;
    }
    
    ret = led_controller_init(led_cmd_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] Failed to initialize LED controller: %s (x_x)", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "[INIT] LED controller initialized (^_^)");

    /* 初始化蓝牙广播管理器 */
    bt_mgr_err_t bt_ret = bluetooth_manager_init();
    if (bt_ret != BT_MGR_OK) {
        ESP_LOGE(TAG, "[ERROR] Failed to initialize bluetooth manager: %d (x_x)", bt_ret);
        return;
    }
    ESP_LOGI(TAG, "[INIT] Bluetooth broadcast manager initialized (^_^)");

    /* 设置回调函数 */
    bluetooth_manager_set_broadcast_callback(on_broadcast_sent);
    bluetooth_manager_set_feedback_callback(on_device_feedback);
    ESP_LOGI(TAG, "[INIT] Callback functions registered (^_^)");

    /* 启动蓝牙广播管理器 */
    bt_ret = bluetooth_manager_start();
    if (bt_ret != BT_MGR_OK) {
        ESP_LOGE(TAG, "[ERROR] Failed to start bluetooth manager: %d (x_x)", bt_ret);
        return;
    }
    ESP_LOGI(TAG, "[START] Bluetooth broadcast manager started (^_^)");

    /* 配置广播参数 */
    bluetooth_manager_set_broadcast_interval(120); // 120ms间隔
    bluetooth_manager_set_repeat_count(3);          // 每个命令重复3次
    ESP_LOGI(TAG, "[CONFIG] Broadcast parameters configured (^_^)");
    
    /* 初始化MQTT管理器 */
    ret = mqtt_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[WARN] Failed to initialize MQTT manager: %s", esp_err_to_name(ret));
        ESP_LOGI(TAG, "[INFO] Continuing without MQTT support (-_-)");
    } else {
        ESP_LOGI(TAG, "[INIT] MQTT manager initialized (^_^)");
    }

    /* 初始化OTA管理器 */
    ret = ota_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[WARN] Failed to initialize OTA manager: %s", esp_err_to_name(ret));
        ESP_LOGI(TAG, "[INFO] Continuing without OTA support (-_-)");
    } else {
        ESP_LOGI(TAG, "[INIT] OTA manager initialized (^_^)");
    }

    /* 创建任务 */
    
    // LED控制器任务
    xTaskCreate(led_command_handler_task, "led_handler", LED_CONTROLLER_STACK_SIZE, led_cmd_queue, 3, NULL);
    ESP_LOGI(TAG, "[TASK] LED controller task created (^_^)");
    
    // 系统状态监控任务
    xTaskCreate(status_monitor_task, "status_monitor", STATUS_MONITOR_STACK_SIZE, NULL, 2, NULL);
    ESP_LOGI(TAG, "[TASK] Status monitor task created (^_^)");
    
    // 广播控制演示任务
    xTaskCreate(broadcast_demo_task, "broadcast_demo", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "[TASK] Broadcast demo task created (^_^)");

    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "    ESP32 Smart Station Ready!         ");
    ESP_LOGI(TAG, "    Broadcasting commands to 5000+ tags ");
    ESP_LOGI(TAG, "    Monitoring system status...         ");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "\n");
    
    // 运行快速测试（可选）
    // 注释掉下面这行如果不需要自动测试
    // quick_test_commands();
}