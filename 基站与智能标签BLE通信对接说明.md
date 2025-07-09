# 基站与智能标签(LED灯条) BLE通信对接说明

## 📋 文档概述

本文档详细说明了ESP32基站与智能LED标签之间的BLE通信协议，供标签端开发人员进行对接开发。

**版本：** V1.0  
**更新日期：** 2024年12月  
**适用场景：** 智能仓库管理系统  

---

## 🏗️ 系统架构

```
APP ↔ MQTT ↔ ESP32基站(Central) ↔ BLE ↔ 智能标签(Peripheral)
```

- **基站角色**：BLE Central（主设备）
- **标签角色**：BLE Peripheral（从设备）
- **通信方式**：基站主动扫描、连接和控制标签
- **最大连接数**：10个标签设备

---

## 📡 BLE基础要求

### 🔧 **硬件要求**
- 支持Bluetooth 4.0及以上
- 支持BLE GATT服务
- 电池供电（CR2450，要求8个月续航）
- 集成LED灯条和蜂鸣器

### 📶 **BLE配置参数**
- **广播间隔**：100-200ms (节能考虑)
- **连接间隔**：建议30-50ms
- **从设备延迟**：0-3
- **监督超时**：4000ms
- **MTU大小**：>=23字节（推荐247字节）

---

## 🎯 设备识别要求

基站通过以下方式识别智能标签：

### 1️⃣ **设备名称识别**
```
设备名称必须以 "SmartTag" 开头
示例：
- SmartTag_001
- SmartTag_A1
- SmartTag_WH_01
```

### 2️⃣ **服务UUID识别**
```
主服务UUID：0x00FF (16位UUID)
在广播包中必须包含此服务UUID
```

### 3️⃣ **广播数据要求**
```c
// 广播数据必须包含：
- 设备名称 (Complete Local Name)
- 服务UUID列表 (包含0x00FF)
- 可选：制造商数据(存储设备ID等信息)
```

---

## 🔌 GATT服务定义

### 📋 **主服务规范**
```
服务UUID：     0x00FF (16位)
服务名称：     LED Control Service
服务类型：     主要服务 (Primary Service)
```

### 📝 **特征值定义**

#### 1️⃣ **命令特征值（Command Characteristic）**
```
特征值UUID：   0xFF01 (16位)
属性：         WRITE, WRITE_NO_RSP
权限：         WRITE
描述：         基站向标签发送控制命令
数据长度：     1-20字节
```

#### 2️⃣ **通知特征值（Notify Characteristic）**
```
特征值UUID：   0xFF02 (16位)
属性：         READ, NOTIFY
权限：         READ
描述：         标签向基站发送状态和响应数据
数据长度：     1-20字节
描述符：       必须支持Client Characteristic Configuration (0x2902)
```

### 🔧 **GATT服务初始化示例**
```c
// 服务定义示例 (Nordic nRF SDK风格)
static ble_uuid_t led_service_uuid = {
    .uuid = 0x00FF,
    .type = BLE_UUID_TYPE_16
};

static ble_gatts_char_handles_t cmd_char_handles;
static ble_gatts_char_handles_t notify_char_handles;

// 特征值属性
ble_gatts_char_md_t cmd_char_md = {
    .char_props.write = 1,
    .char_props.write_wo_resp = 1
};

ble_gatts_char_md_t notify_char_md = {
    .char_props.read = 1,
    .char_props.notify = 1
};
```

---

## 📦 通信协议格式

### 🏷️ **协议帧结构**
```
┌────────────────────────────────────────────────────────────┐
│ 帧头 │ 长度 │      数据部分      │ 帧尾 │
│ AA55 │  L   │    Data(0-18)     │ 55AA │
├──────┼──────┼───────────────────┼──────┤
│ 2字节│ 1字节│     L字节         │ 2字节│
└────────────────────────────────────────────────────────────┘
总长度：5 + L 字节 (最大23字节)
```

### 🔧 **协议帧处理函数**
```c
// 协议帧打包函数 - 简化版本（无校验位）
uint16_t protocol_frame_pack(uint8_t *frame_buf, const uint8_t *data, uint16_t data_len) {
    if (!frame_buf || !data || data_len > MAX_PROTOCOL_DATA_LENGTH) {
        return 0;
    }
    
    frame_buf[0] = 0xAA;                    // 帧头1
    frame_buf[1] = 0x55;                    // 帧头2
    frame_buf[2] = data_len;                // 数据长度
    
    // 复制数据
    memcpy(&frame_buf[3], data, data_len);
    
    frame_buf[3 + data_len] = 0x55;         // 帧尾1
    frame_buf[4 + data_len] = 0xAA;         // 帧尾2
    
    return 5 + data_len;                    // 返回总帧长度
}

// 协议帧解包函数 - 简化版本（无校验位）
uint16_t protocol_frame_unpack(const uint8_t *frame_buf, uint16_t frame_len, 
                              uint8_t *data_buf, uint16_t data_buf_size) {
    if (!frame_buf || !data_buf || frame_len < 5) {
        return 0;
    }
    
    // 检查帧头
    if (frame_buf[0] != 0xAA || frame_buf[1] != 0x55) {
        return 0;  // 帧头错误
    }
    
    uint8_t data_len = frame_buf[2];
    
    // 检查长度合法性
    if (data_len > MAX_PROTOCOL_DATA_LENGTH || (5 + data_len) != frame_len) {
        return 0;  // 长度错误
    }
    
    // 检查帧尾
    if (frame_buf[3 + data_len] != 0x55 || frame_buf[4 + data_len] != 0xAA) {
        return 0;  // 帧尾错误
    }
    
    // 复制数据
    if (data_len <= data_buf_size) {
        memcpy(data_buf, &frame_buf[3], data_len);
        return data_len;
    }
    
    return 0;  // 缓冲区不足
}
```

### 🎯 **数据部分格式**

#### 📤 **基站→标签命令格式**
```c
typedef struct __attribute__((packed)) {
    uint8_t cmd_type;        // 命令类型
    uint8_t data_length;     // 数据长度
    uint16_t device_id;      // 设备ID(大端格式)
    uint8_t cmd_data[14];    // 命令数据(最大14字节)
} station_to_strip_cmd_t;
```

#### 📥 **标签→基站响应格式**
```c
typedef struct __attribute__((packed)) {
    uint8_t cmd_type;        // 命令类型/响应类型
    uint16_t device_id;      // 设备ID(大端格式)
    uint8_t data[15];        // 数据内容(最大15字节)
} strip_to_station_data_t;
```

---

## 🎨 命令类型详细说明

### 1️⃣ **设置亮灯颜色 (0x02)**
```c
// 基站发送
struct {
    uint8_t cmd_type = 0x02;     // 命令类型
    uint8_t data_length = 6;     // 数据长度
    uint16_t device_id;          // 设备ID
    uint8_t color;               // 颜色值(见颜色定义)
    uint8_t brightness;          // 亮度(0-100)
    uint32_t duration_ms;        // 持续时间(毫秒，0=永久)
} set_color_cmd;

// 标签响应
struct {
    uint8_t cmd_type = 0x02;     // 响应类型
    uint16_t device_id;          // 设备ID
    uint8_t status;              // 执行状态(0=成功)
    uint8_t current_color;       // 当前颜色
    uint8_t current_brightness;  // 当前亮度
} set_color_rsp;
```

### 2️⃣ **设置闪烁模式 (0x03)**
```c
// 基站发送
struct {
    uint8_t cmd_type = 0x03;     // 命令类型
    uint8_t data_length = 4;     // 数据长度
    uint16_t device_id;          // 设备ID
    uint8_t blink_mode;          // 闪烁模式(见模式定义)
    uint8_t color;               // 闪烁颜色
    uint16_t duration_ms;        // 持续时间(毫秒)
} set_blink_cmd;
```

### 3️⃣ **设置蜂鸣器状态 (0x04)**
```c
// 基站发送
struct {
    uint8_t cmd_type = 0x04;     // 命令类型
    uint8_t data_length = 3;     // 数据长度
    uint16_t device_id;          // 设备ID
    uint8_t beep_enable;         // 蜂鸣器开关(0=关，1=开)
    uint16_t duration_ms;        // 蜂鸣持续时间(毫秒)
} set_beep_cmd;
```

### 4️⃣ **获取电池状态 (0x06)**
```c
// 基站发送
struct {
    uint8_t cmd_type = 0x06;     // 命令类型
    uint8_t data_length = 2;     // 数据长度
    uint16_t device_id;          // 设备ID
} get_battery_cmd;

// 标签响应
struct {
    uint8_t cmd_type = 0x06;     // 响应类型
    uint16_t device_id;          // 设备ID
    uint8_t battery_level;       // 电池电量等级(0-4)
    uint16_t voltage_mv;         // 电池电压(毫伏)
    uint8_t charge_status;       // 充电状态
} battery_status_rsp;
```

### 5️⃣ **电池低电量警报 (0x07)**
```c
// 标签主动发送
struct {
    uint8_t cmd_type = 0x07;     // 警报类型
    uint16_t device_id;          // 设备ID
    uint8_t battery_level;       // 当前电量等级
    uint16_t voltage_mv;         // 当前电压(毫伏)
    uint8_t warning_level;       // 警告等级(0=低，1=严重)
} battery_alert;
```

---

## 🎨 颜色和模式定义

### 🌈 **LED颜色定义**
```c
#define LED_COLOR_RED     0x00    // 红色
#define LED_COLOR_GREEN   0x01    // 绿色
#define LED_COLOR_BLUE    0x02    // 蓝色
#define LED_COLOR_YELLOW  0x03    // 黄色
#define LED_COLOR_PURPLE  0x04    // 紫色
#define LED_COLOR_CYAN    0x05    // 青色
#define LED_COLOR_WHITE   0x06    // 白色
#define LED_COLOR_OFF     0x07    // 关闭
```

### ⚡ **闪烁模式定义**
```c
#define BLINK_MODE_NONE   0x00    // 不闪烁(常亮)
#define BLINK_MODE_SLOW   0x01    // 慢闪(1Hz)
#define BLINK_MODE_FAST   0x02    // 快闪(2Hz)
```

### 🔋 **电池电量等级**
```c
#define BATTERY_LEVEL_CRITICAL  0x00  // 严重不足(<10%)
#define BATTERY_LEVEL_LOW       0x01  // 低(10-25%)
#define BATTERY_LEVEL_MEDIUM    0x02  // 中等(25-50%)
#define BATTERY_LEVEL_HIGH      0x03  // 高(50-75%)
#define BATTERY_LEVEL_FULL      0x04  // 满电(>75%)
```

---

## 🔄 通信流程

### 📡 **连接建立流程**
```
1. 标签启动BLE广播
   ├─ 广播设备名称 "SmartTag_xxx"
   ├─ 广播服务UUID 0x00FF
   └─ 广播间隔：100-200ms

2. 基站扫描发现标签
   ├─ 扫描持续时间：10秒
   ├─ 扫描间隔：15秒
   └─ 识别目标设备

3. 基站发起连接
   ├─ 连接超时：5秒
   ├─ 分配设备ID (1-10)
   └─ 保存设备信息

4. GATT服务发现
   ├─ 发现主服务 0x00FF
   ├─ 发现命令特征值 0xFF01
   └─ 发现通知特征值 0xFF02

5. 启用通知
   ├─ 写入CCCD描述符
   └─ 启用状态上报
```

### ⚡ **命令执行流程**
```
1. 基站发送命令
   ├─ 构建协议帧
   ├─ 写入命令特征值
   └─ 等待响应

2. 标签处理命令
   ├─ 解析协议帧
   ├─ 验证校验和
   ├─ 执行命令操作
   └─ 构建响应数据

3. 标签发送响应
   ├─ 通过通知特征值
   ├─ 发送执行结果
   └─ 可选状态更新

4. 基站处理响应
   ├─ 验证响应数据
   ├─ 更新设备状态
   └─ 触发回调函数
```

---

## 🧪 测试验证方法

### 1️⃣ **设备识别测试**
```c
// 确保标签能被基站发现
// 检查日志输出：
[SCAN] ===== BLE SCAN RESULT =====
[DEV] Device Name: SmartTag_xxx
[MAC] MAC Address: xx:xx:xx:xx:xx:xx
[TYPE] Is Smart Tag: YES ^_^
```

### 2️⃣ **连接测试**
```c
// 确保连接建立成功
// 检查日志输出：
[CONN] Found target Smart Tag! Attempting connection...
[OK] Connection request sent successfully ^_^
[OK] Device ID 1: Successfully connected! \(^o^)/
```

### 3️⃣ **命令响应测试**
```c
// 基站会自动发送绿色测试命令
// 标签应该：
1. 点亮绿色LED 3秒钟
2. 发送执行成功响应
3. 基站显示：[SEND] Test command sent successfully
```

### 4️⃣ **协议解析测试**
```c
// 验证协议帧格式
uint8_t test_frame[] = {
    0xAA, 0x55,           // 帧头
    0x06,                 // 数据长度
    0x02, 0x06, 0x00, 0x01, 0x01, 0x00, // 数据部分
    0x55, 0xAA            // 帧尾
};
```

---

## 🚨 错误处理

### ⚠️ **常见错误代码**
```c
#define PROTOCOL_OK                 0x00  // 成功
#define PROTOCOL_ERR_FRAME_HEAD     0x01  // 帧头错误
#define PROTOCOL_ERR_FRAME_TAIL     0x02  // 帧尾错误
#define PROTOCOL_ERR_LENGTH         0x03  // 长度错误
#define PROTOCOL_ERR_BUFFER_FULL    0x04  // 缓冲区满
```

### 🔧 **故障排除指南**
1. **设备无法被发现**
   - 检查设备名称格式
   - 检查服务UUID广播
   - 检查广播间隔设置

2. **连接失败**
   - 检查连接参数配置
   - 确认设备可连接状态
   - 检查信号强度

3. **命令无响应**
   - 验证GATT特征值UUID
   - 检查通知特征值是否启用
   - 验证协议帧格式

4. **数据传输错误**
   - 检查MTU大小配置
   - 确认数据长度正确
   - 验证帧头帧尾格式

---

## 💡 开发建议

### 🔋 **功耗优化**
- 合理设置广播间隔（推荐100-200ms）
- 在无连接时进入低功耗模式
- 优化连接参数减少功耗
- 定期上报电池状态

### 📶 **通信稳定性**
- 实现重连机制
- 添加心跳包检测
- 处理连接丢失情况
- 缓存未发送数据

### 🛡️ **可靠性保证**
- 实现帧头帧尾验证
- 添加命令确认机制
- 处理异常断开
- 记录关键状态

---

## 📞 技术支持

如有对接过程中的技术问题，请联系：
- **项目负责人**：ESP32基站开发团队
- **技术文档**：参考本说明文档
- **测试环境**：提供基站测试固件

---

**📄 版权说明**：本文档仅供项目开发使用，请勿外传。 