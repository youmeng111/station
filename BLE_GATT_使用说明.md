# ESP32 基站 BLE Central 使用说明 - 灯条控制系统 V3.0

基于 ESP-IDF v5.4.1 NimBLE Central 开发的智能基站系统，作为APP与多个LED灯条设备之间的蓝牙网关。

## 概述

本项目实现了一个ESP32基站，使用BLE Central模式连接和管理多个LED灯条设备。基站同时支持WiFi+MQTT通信，作为移动APP和灯条设备之间的双向通信网关。

## 系统架构

```
[移动APP] <-> [MQTT平台] <-> [WiFi] <-> [ESP32基站] <-> [BLE] <-> [灯条设备1-10]
```

### 核心功能
- **BLE Central**: 主动扫描和连接多个灯条设备（最多10个）
- **设备管理**: 自动发现、连接管理、状态监控
- **MQTT网关**: 接收APP指令，转发给对应灯条设备
- **双向通信**: 收集灯条状态，上报给MQTT平台
- **灯条协议**: 使用标准灯条协议V2.0进行BLE通信

## 特性

- **双重角色**: BLE Central + MQTT Client
- **多设备管理**: 支持同时连接10个灯条设备
- **自动重连**: 设备断线自动重连机制
- **命令转发**: MQTT命令自动转换为灯条协议
- **状态上报**: 实时收集和上报设备状态
- **广播支持**: 支持向所有设备广播命令

## 项目结构

```
components/bluetooth_manager/
├── include/
│   ├── bluetooth_manager.h    # BLE Central接口
│   └── common.h              # 协议定义和公共类型
├── bluetooth_manager.c       # BLE Central实现
└── CMakeLists.txt          # 组件构建配置

main/
├── main.c                   # 主程序入口
└── CMakeLists.txt

components/mqtt_manager/     # MQTT客户端管理
components/led_controller/   # LED状态管理
components/system_config/    # 系统配置管理
components/ota_manager/      # OTA升级管理
```

## 设备发现和连接

### 目标设备识别
基站通过设备名称前缀识别灯条设备：
- **设备名前缀**: `SmartTag`
- **服务UUID**: `0x00FF` (LED控制服务)

### 连接管理
- **最大连接数**: 10个设备
- **设备ID范围**: 1-10
- **连接超时**: 5秒
- **扫描周期**: 10秒（无设备时）

## 灯条协议通信

### BLE GATT特征值

| 特征值 | UUID | 用途 | 方向 |
|-------|------|------|------|
| 命令特征 | `0xFF01` | 发送控制命令 | 基站→灯条 |
| 通知特征 | `0xFF02` | 接收状态响应 | 灯条→基站 |

### 协议帧格式

基站使用标准灯条协议V2.0：
```
[帧头AA55] [长度] [命令数据] [校验_H] [校验_L] [帧尾55AA]
```

## MQTT命令接口

### 1. 设备管理命令

#### 扫描设备
```json
{"cmd":"SCAN_DEVICES"}
```

#### 连接设备
```json
{
  "cmd":"CONNECT_DEVICE",
  "mac":"aa:bb:cc:dd:ee:ff",
  "device_id":1
}
```

#### 断开设备
```json
{
  "cmd":"DISCONNECT_DEVICE",
  "device_id":1
}
```

### 2. LED控制命令

#### 单设备颜色控制
```json
{
  "cmd":"LED_CONTROL",
  "device_id":1,
  "color":"RED",
  "duration":5000
}
```

#### 单设备开关
```json
{"cmd":"LED_ON", "device_id":1}
{"cmd":"LED_OFF", "device_id":1}
```

#### 广播控制
```json
{
  "cmd":"BROADCAST",
  "color":"BLUE",
  "duration":3000
}
```

### 3. 状态查询
```json
{"cmd":"GET_STATUS"}
```

## 状态上报格式

基站定期上报系统状态：
```json
{
  "device_type":"station",
  "connected_devices":3,
  "max_devices":10,
  "bt_state":3,
  "system":{
    "wifi":true,
    "mqtt":true,
    "bt_central":"ready",
    "uptime":12345
  }
}
```

## API接口

### 初始化和启动
```c
// 初始化BLE Central
bt_mgr_err_t bluetooth_manager_init(void);

// 启动基站服务
bt_mgr_err_t bluetooth_manager_start(void);

// 设置回调函数
bluetooth_manager_set_state_callback(device_state_cb);
bluetooth_manager_set_response_callback(device_response_cb);
```

### 设备扫描和连接
```c
// 开始扫描
bt_mgr_err_t bluetooth_manager_start_scan(
    uint32_t duration_ms, 
    device_found_cb_t found_cb
);

// 连接设备
bt_mgr_err_t bluetooth_manager_connect_device(
    const uint8_t *mac_addr, 
    uint8_t device_id
);
```

### 命令发送
```c
// 发送命令到指定设备
bt_mgr_err_t bluetooth_manager_send_command(
    uint8_t device_id, 
    const led_command_t *cmd
);

// 广播命令到所有设备
int bluetooth_manager_broadcast_command(const led_command_t *cmd);
```

### 状态查询
```c
// 获取连接的设备数量
int bluetooth_manager_get_connected_count(void);

// 获取设备信息
const led_device_t* bluetooth_manager_get_device(uint8_t device_id);

// 检查设备是否在线
bool bluetooth_manager_is_device_online(uint8_t device_id);
```

## 设备状态管理

### 设备状态枚举
```c
typedef enum {
    LED_DEVICE_STATE_DISCONNECTED = 0,
    LED_DEVICE_STATE_CONNECTING,
    LED_DEVICE_STATE_CONNECTED,
    LED_DEVICE_STATE_ERROR
} led_device_state_t;
```

### 设备信息结构
```c
typedef struct {
    uint8_t device_id;              // 设备ID (1-10)
    uint8_t mac_addr[6];           // MAC地址
    char name[32];                 // 设备名称
    uint16_t conn_handle;          // 连接句柄
    led_device_state_t state;      // 连接状态
    int8_t rssi;                   // 信号强度
    led_color_t current_color;     // 当前颜色
    bool is_on;                    // 是否开启
    uint8_t battery_level;         // 电池电量
} led_device_t;
```

## 回调函数

### 设备发现回调
```c
void on_device_found(const scan_result_t *result) {
    ESP_LOGI(TAG, "Found LED device: %s, RSSI: %d", 
             result->name, result->rssi);
    // 可以实现自动连接逻辑
}
```

### 状态变化回调
```c
void on_device_state_changed(uint8_t device_id, 
                            led_device_state_t old_state, 
                            led_device_state_t new_state) {
    if (new_state == LED_DEVICE_STATE_CONNECTED) {
        ESP_LOGI(TAG, "Device %d connected", device_id);
    }
}
```

## 部署配置

### ESP-IDF配置
```bash
# 启用BLE Central
CONFIG_BT_NIMBLE_ROLE_CENTRAL=y
CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=n

# WiFi配置
CONFIG_ESP_WIFI_ENABLED=y

# 分区表
CONFIG_PARTITION_TABLE_CUSTOM=y
```

### WiFi和MQTT配置
在 `components/system_config/include/system_config.h` 中配置：
```c
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"
#define MQTT_BROKER_URL "mqtt://your_broker_ip"
#define MQTT_USERNAME "your_username"
#define MQTT_PASSWORD "your_password"
```

## 使用流程

### 1. 基站启动
1. 初始化WiFi和MQTT连接
2. 启动BLE Central服务
3. 自动开始扫描LED设备

### 2. 设备连接
1. APP发送扫描命令：`{"cmd":"SCAN_DEVICES"}`
2. 基站发现设备后，APP选择连接：`{"cmd":"CONNECT_DEVICE",...}`
3. 基站建立BLE连接并上报状态

### 3. 设备控制
1. APP发送控制命令：`{"cmd":"LED_CONTROL",...}`
2. 基站转换为灯条协议并通过BLE发送
3. 收集设备响应并通过MQTT上报

## 故障排除

### 常见问题
1. **设备扫描不到**: 检查设备名称前缀匹配
2. **连接失败**: 检查设备是否已连接到其他基站
3. **命令无响应**: 检查设备ID和连接状态
4. **MQTT断线**: 检查WiFi和MQTT配置

### 调试日志
```c
// 启用调试日志
esp_log_level_set("ESP32_Station_Central", ESP_LOG_DEBUG);
esp_log_level_set("MQTT_MANAGER", ESP_LOG_DEBUG);
```

## 版本信息

- **架构版本**: 3.0 (BLE Central + MQTT Gateway)
- **协议版本**: 2.0 (兼容标准灯条协议)
- **ESP-IDF版本**: v5.4.1
- **最大设备数**: 10个
- **UUID格式**: 16位自定义UUID

## 版本历史

- **v2.2**: 将CRC16校验替换为16位和校验，提升计算效率和调试友好性
- **v2.1**: 优化为16位UUID，解决广播数据超限问题
- **v2.0**: 灯条协议规范兼容版本
- **ESP-IDF版本**: v5.4.1
- **NimBLE栈**: 包含在ESP-IDF中 