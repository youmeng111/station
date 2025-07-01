# ESP32 Station BLE GATT 服务使用说明 - 灯条协议 V2.0

基于 ESP-IDF v5.4.1 NimBLE GATT Server 例程开发的LED灯条控制系统，采用统一的灯条通信协议规范。

## 概述

本项目实现了一个完整的BLE GATT服务器，用于通过蓝牙控制多个LED灯条。使用灯条协议规范V2.0，支持**128位UUID**、**CRC16校验**和**帧头帧尾**保护机制。

## 特性

- **NimBLE栈**: 使用ESP-IDF的NimBLE蓝牙栈
- **模块化设计**: 清晰分离GAP、GATT、管理器功能
- **灯条协议V2.0**: 符合统一的灯条通信协议规范
- **128位UUID**: 使用自定义128位UUID服务
- **CRC16校验**: 使用CRC-16-IBM算法确保数据完整性
- **多LED支持**: 支持控制多个LED灯条
- **实时通知**: 支持状态变化实时通知
- **错误处理**: 完善的错误处理和日志记录

## 项目结构

```
components/bluetooth_manager/
├── include/
│   ├── bluetooth_manager.h    # 主接口头文件
│   ├── common.h              # 公共定义
│   ├── gap.h                 # GAP服务头文件
│   └── gatt_svc.h           # GATT服务头文件
├── bluetooth_manager.c       # 主管理器实现
├── gap.c                    # GAP服务实现
├── gatt_svc.c              # GATT服务实现
└── CMakeLists.txt          # 组件构建配置
```

## GATT服务定义

### 服务UUID (128位)
- **无线灯条控制服务**: `12345678-90AB-CDEF-1234-567890ABCDEF`

### 特征值 (128位UUID)

| 特征值 | UUID | 权限 | 描述 |
|-------|------|------|------|
| 命令特征 | `12345678-90AB-CDEF-1234-567890ABCDE1` | Write, Write Without Response | 接收灯条协议命令 |
| 响应特征 | `12345678-90AB-CDEF-1234-567890ABCDE2` | Notify | 发送状态和响应数据 |

## 灯条协议格式

### 协议帧结构

```
[帧头AA55] [长度] [数据0-18] [CRC16_H] [CRC16_L] [帧尾55AA]
```

| 字段     | 长度(字节) | 说明                              |
|----------|------------|-----------------------------------|
| 帧头     | 2          | 固定为 AA 55                      |
| 长度     | 1          | 数据部分的长度(不包含帧头、长度、CRC、帧尾) |
| 数据     | 0-18       | 实际数据内容                      |
| CRC16    | 2          | 数据部分的CRC16校验(大端格式)      |
| 帧尾     | 2          | 固定为 55 AA                      |

### CRC16校验算法

- **多项式**: 0x8005 (CRC-16-IBM)
- **初始值**: 0x0000
- **校验范围**: 仅对数据部分进行校验
- **字节序**: 大端格式(高字节在前)

## 命令类型定义

| 命令类型 | 值 | 说明 |
|---------|---|------|
| CMD_SET_LIGHT_DURATION | 0x01 | 设置亮灯时长 |
| CMD_SET_LIGHT_COLOR | 0x02 | 设置亮灯颜色 |
| CMD_SET_BLINK_MODE | 0x03 | 设置闪烁模式 |
| CMD_SET_BEEP_STATE | 0x04 | 设置蜂鸣器状态 |
| CMD_SET_CONTROL_MODE | 0x05 | 设置控制模式 |
| CMD_GET_BATTERY_STATUS | 0x06 | 获取/上报电池状态 |
| CMD_BATTERY_LOW_ALERT | 0x07 | 电池电量低警报 |

### 颜色值定义

| 颜色 | 值 | RGB映射 |
|-----|---|---------|
| 红色 | 0x00 | 255,0,0 |
| 绿色 | 0x01 | 0,255,0 |
| 蓝色 | 0x02 | 0,0,255 |
| 黄色 | 0x03 | 255,255,0 |
| 紫色 | 0x04 | 255,0,255 |
| 青色 | 0x05 | 0,255,255 |
| 白色 | 0x06 | 255,255,255 |
| 关闭 | 0x07 | LED_OFF |

## 命令示例

### 设置LED颜色为红色
**基站发送:**
```
数据部分: 02 01 00 01 00
完整帧: AA 55 05 02 01 00 01 00 40 C3 55 AA
```

### 设置亮灯时长30秒
**基站发送:**
```
数据部分: 01 02 00 01 00 1E
完整帧: AA 55 06 01 02 00 01 00 1E 8C 3A 55 AA
```

## 协议优势

1. **标准化**: 统一的协议规范，与其他灯条设备兼容
2. **可靠性**: CRC16校验确保数据完整性
3. **扩展性**: 支持多种命令类型和参数
4. **兼容性**: 128位UUID避免与其他设备冲突
5. **调试友好**: 帧头帧尾便于协议调试

## 版本信息

- **协议版本**: 2.0 (与灯条协议规范兼容)
- **ESP-IDF版本**: v5.4.1
- **NimBLE栈**: 包含在ESP-IDF中
- **UUID格式**: 128位自定义UUID

## 使用方法

### 1. 初始化

```c
#include "bluetooth_manager.h"

// 初始化蓝牙管理器
bt_mgr_err_t ret = bluetooth_manager_init();
if (ret != BT_MGR_OK) {
    ESP_LOGE(TAG, "Failed to initialize Bluetooth manager");
    return;
}

// 启动蓝牙服务
ret = bluetooth_manager_start();
if (ret != BT_MGR_OK) {
    ESP_LOGE(TAG, "Failed to start Bluetooth manager");
    return;
}
```

### 2. 状态监控

```c
// 检查连接状态
if (bluetooth_manager_is_connected()) {
    // 发送状态通知
    bluetooth_manager_notify_led_status();
}

// 获取管理器状态
bt_mgr_state_t state = bluetooth_manager_get_state();
```

### 3. 发送响应

```c
// 发送自定义响应
bluetooth_manager_send_response("success", "Operation completed");
```

## 客户端连接流程

1. **扫描设备**: 查找名为 "ESP32_Station_BLE" 的设备
2. **建立连接**: 连接到设备
3. **服务发现**: 发现LED控制服务 (UUID: 0x00FF)
4. **启用通知**: 订阅状态和响应特征值的通知
5. **发送命令**: 向命令特征值写入二进制命令
6. **接收响应**: 通过通知接收命令执行结果

## 校验和计算

使用XOR校验：
```c
uint8_t calculate_checksum(const uint8_t *data, uint16_t length) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
```

## 测试工具

推荐使用以下工具进行测试：

- **nRF Connect for Mobile** (Android/iOS)
- **LightBlue Explorer** (iOS)
- **BLE Scanner** (Android)
- **自定义测试脚本** (Python + bleak库)

## 配置选项

在 `sdkconfig.defaults` 中的关键配置：

```
# 启用BLE
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y

# 禁用经典蓝牙
CONFIG_BT_CLASSIC_ENABLED=n

# 启用NimBLE GATT服务器
CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=y
CONFIG_BT_NIMBLE_ROLE_CENTRAL=n
```

## 故障排除

1. **校验和错误**: 检查数据传输是否完整
2. **无效命令**: 确认命令类型和参数格式正确
3. **连接问题**: 检查BLE连接状态和通知订阅
4. **数据长度**: 确保数据长度字段与实际参数长度一致

## 版本信息

- **协议版本**: 1.0
- **ESP-IDF版本**: v5.4.1
- **NimBLE栈**: 包含在ESP-IDF中 