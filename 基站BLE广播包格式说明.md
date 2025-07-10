 # ESP32基站BLE广播包格式说明文档

## 📋 文档概述

**版本：** V2.0 (广播架构 - 无校验版本)  
**更新日期：** 2024年12月  
**适用场景：** 快递驿站智能标签管理系统  
**基站型号：** ESP32-S3  
**支持设备数量：** 5000+ 智能标签  

---

## 🏗️ 系统架构变更

### 原架构 vs 新架构

| 特性 | 原GATT架构 | 新广播架构 |
|------|------------|------------|
| 连接方式 | 点对点连接 | 一对多广播 |
| 最大设备数 | 10台 | 5000+台 |
| 功耗 | 高（维持连接） | 低（间歇监听） |
| 延迟 | 高（连接建立） | 低（<10ms） |
| 可靠性 | 连接依赖 | 重发机制 |

### 通信模式
```
基站 --BLE广播--> 所有标签（单向）
- 无需配对连接
- 无需维持连接状态
- 支持分组控制
- 支持优先级管理
```

---

## 📦 BLE广播包结构

### 总体格式 (无校验版本)
```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ BLE AD Header │        Manufacturer Data         │
├───────────────┼───────────────────────────────────┤
│   AD Length   │   AD Type   │   Company ID   │   Custom Protocol Data   │
│   (1 byte)    │   (1 byte)  │   (2 bytes)    │      (17 bytes)          │
└─────────────────────────────────────────────────────────────────────────────────┘
总长度：21字节 (符合BLE广播31字节限制，去掉校验和字段)
```

### 详细字段布局 (无校验版本)
```
Byte:  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20
     ┌────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┐
     │ 14 │ FF │ E5 │ 02 │ AA │ 55 │ CT │ GI │ SL │ SH │ PR │ P1 │ P2 │ P3 │ P4 │ P5 │ P6 │ P7 │ P8 │ 99 │
     └────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┘
```

---

## 🔧 字段详细说明

### 1. BLE AD结构字段

| 字节位置 | 字段名称 | 值 | 说明 |
|----------|----------|-----|------|
| 0 | AD Length | 0x14 (20) | 广播数据长度（不包括长度字段本身） |
| 1 | AD Type | 0xFF | 厂商数据类型 (Manufacturer Specific Data) |
| 2 | Company ID Low | 0xE5 | Espressif厂商ID低字节 |
| 3 | Company ID High | 0x02 | Espressif厂商ID高字节 (0x02E5) |

### 2. 自定义协议字段

| 字节位置 | 字段名称 | 长度 | 说明 |
|----------|----------|------|------|
| 4-5 | Protocol Header | 2字节 | 协议头标识 {0xAA, 0x55} |
| 6 | Command Type | 1字节 | 命令类型 (0x01-0x08) |
| 7 | Group ID | 1字节 | 目标分组ID (0x01-0xFF) |
| 8-9 | Sequence Number | 2字节 | 序列号 (小端序) |
| 10 | Priority | 1字节 | 优先级 (0x00-0x03) |
| 11-18 | Parameters | 8字节 | 命令参数数据 |
| 19 | Protocol Tail | 1字节 | 协议尾标识 (0x99) |

---

## 🎯 命令类型定义

### 基础控制命令

| 命令类型 | 值 | 命令名称 | 说明 |
|----------|-----|----------|------|
| `BROADCAST_CMD_SET_COLOR` | 0x01 | 设置颜色 | 设置LED颜色和亮度 |
| `BROADCAST_CMD_SET_BLINK` | 0x02 | 设置闪烁 | 设置闪烁模式 |
| `BROADCAST_CMD_SET_BEEP` | 0x03 | 设置蜂鸣器 | 控制蜂鸣器状态 |
| `BROADCAST_CMD_GROUP_ACTIVATE` | 0x04 | 分组激活 | 激活特定分组 |
| `BROADCAST_CMD_ALL_OFF` | 0x05 | 全部关闭 | 关闭所有设备 |
| `BROADCAST_CMD_BATTERY_QUERY` | 0x06 | 电池查询 | 查询电池状态 |
| `BROADCAST_CMD_STATUS_SYNC` | 0x07 | 状态同步 | 同步设备状态 |
| `BROADCAST_CMD_BATTERY_WARNING` | 0x08 | 电池预警 | 智能标签低电预警 |
| `BROADCAST_CMD_INVENTORY_SCAN` | 0x09 | 盘点扫描 | 发起标签盘点扫描，收集标签状态信息 |
| `BROADCAST_CMD_HEALTH_CHECK` | 0x0A | 健康检查 | 检查标签设备健康状态（电池、LED、蜂鸣器等） |
| `BROADCAST_CMD_FAULT_REPORT` | 0x0B | 故障上报 | 标签设备故障信息上报 |

### 命令参数格式

#### 1. 设置颜色命令 (0x01)
```c
struct {
    uint8_t color;          // 颜色值 (0x00-0x07)
    uint8_t blink_mode;     // 闪烁模式 (0x00-0x03)
    uint8_t beep_mode;      // 蜂鸣器模式 (0x00-0x03)
    uint32_t duration_ms;   // 持续时间(毫秒，小端序)
    uint8_t reserved;       // 保留字节
} led_control_params;
```

#### 2. 电池预警命令 (0x08)
```c
struct {
    uint8_t battery_level;  // 电池电量百分比
    uint16_t voltage_mv;    // 电池电压(毫伏)
    uint8_t warning_type;   // 预警类型 (0x01=低电, 0x02=极低电)
    uint8_t reserved[4];    // 保留字节
} battery_warning_params;
```

#### 3. 盘点扫描命令 (0x09)
```c
struct {
    uint32_t scan_id;       // 盘点ID
    uint8_t scan_type;      // 盘点类型 (0=全部, 1=分组, 2=指定)
    uint32_t timeout_ms;    // 响应超时时间(毫秒)
    uint8_t reserved[0];    // 保留字节
} inventory_scan_params;
```

#### 4. 健康检查命令 (0x0A)
```c
struct {
    uint8_t check_items;    // 检查项目位掩码
    uint32_t check_id;      // 检查ID
    uint8_t reserved[3];    // 保留字节
} health_check_params;
```

#### 5. 故障上报命令 (0x0B)
```c
struct {
    uint32_t tag_id;        // 标签ID
    uint8_t fault_type;     // 故障类型
    uint8_t fault_code;     // 故障代码
    uint8_t reserved[2];    // 保留字节
} fault_report_params;
```

---

## 🏷️ 分组管理

### 预定义分组

| 分组ID | 分组名称 | 说明 | 预估设备数 |
|--------|----------|------|------------|
| 0xFF | GROUP_ALL | 所有设备 | 5000+ |
| 0x01 | GROUP_SHELF_A | 货架A区域 | 1000 |
| 0x02 | GROUP_SHELF_B | 货架B区域 | 1000 |
| 0x03 | GROUP_SHELF_C | 货架C区域 | 1000 |
| 0x04 | GROUP_SHELF_D | 货架D区域 | 1000 |
| 0x10 | GROUP_COUNTER | 取件台区域 | 200 |
| 0x20 | GROUP_ENTRANCE | 入口区域 | 100 |
| 0x30 | GROUP_SPECIAL | 特殊包裹区域 | 50 |
| 0xF0 | GROUP_MAINTENANCE | 维护模式组 | 10 |

### 分组使用示例
```c
// 控制货架A区域所有设备变红色（过期包裹）
group_id = 0x01;  // GROUP_SHELF_A
cmd_type = 0x01;  // SET_COLOR
color = 0x01;     // RED
```

---

## 🎨 颜色和模式定义

### LED颜色定义
```c
#define LED_COLOR_OFF       0x00    // 关闭
#define LED_COLOR_RED       0x01    // 红色
#define LED_COLOR_GREEN     0x02    // 绿色
#define LED_COLOR_BLUE      0x03    // 蓝色
#define LED_COLOR_YELLOW    0x04    // 黄色
#define LED_COLOR_PURPLE    0x05    // 紫色
#define LED_COLOR_CYAN      0x06    // 青色
#define LED_COLOR_WHITE     0x07    // 白色
```

### 闪烁模式定义
```c
#define BLINK_MODE_NONE     0x00    // 常亮
#define BLINK_MODE_SLOW     0x01    // 慢闪 (1Hz)
#define BLINK_MODE_FAST     0x02    // 快闪 (3Hz)
#define BLINK_MODE_URGENT   0x03    // 紧急闪烁 (5Hz)
```

### 蜂鸣器模式定义
```c
#define BEEP_MODE_OFF       0x00    // 关闭
#define BEEP_MODE_SINGLE    0x01    // 单次哔声
#define BEEP_MODE_DOUBLE    0x02    // 双次哔声
#define BEEP_MODE_CONTINUOUS 0x03   // 连续哔声
```

### 优先级定义
```c
#define PRIORITY_LOW        0x00    // 低优先级
#define PRIORITY_NORMAL     0x01    // 普通优先级
#define PRIORITY_HIGH       0x02    // 高优先级
#define PRIORITY_EMERGENCY  0x03    // 紧急优先级
```

---

## 📊 实际广播包示例

### 示例1：全部设备变绿色 (无校验版本)
```
原始数据：14 FF E5 02 AA 55 01 FF 01 00 01 02 00 00 00 00 0A 00 00 99
解析：
- 长度: 0x14 (20字节)
- 类型: 0xFF (厂商数据)
- 厂商ID: 0x02E5 (Espressif)
- 协议头: 0xAA55
- 命令类型: 0x01 (SET_COLOR)
- 分组ID: 0xFF (ALL)
- 序列号: 0x0001
- 优先级: 0x01 (NORMAL)
- 颜色: 0x02 (GREEN)
- 闪烁: 0x00 (NONE)
- 蜂鸣: 0x00 (OFF)
- 持续时间: 0x00000A00 (2560ms)
- 协议尾: 0x99
```

### 示例2：货架A区域包裹取件提醒
```
原始数据：14 FF E5 02 AA 55 01 01 02 00 02 04 01 00 00 75 30 00 00 99
解析：
- 命令类型: 0x01 (SET_COLOR)
- 分组ID: 0x01 (SHELF_A)
- 序列号: 0x0002
- 优先级: 0x02 (HIGH)
- 颜色: 0x04 (YELLOW)
- 闪烁: 0x01 (SLOW)
- 蜂鸣: 0x00 (OFF)
- 持续时间: 0x00007530 (30000ms = 30秒)
```

### 示例3：智能标签电池预警
```
原始数据：14 FF E5 02 AA 55 08 FF 03 00 02 14 F0 0A 01 00 00 00 00 99
解析：
- 命令类型: 0x08 (BATTERY_WARNING)
- 分组ID: 0xFF (ALL)
- 序列号: 0x0003
- 优先级: 0x02 (HIGH)
- 电池电量: 0x14 (20%)
- 电池电压: 0x0AF0 (2800mV)
- 预警类型: 0x01 (低电预警)
```

---

## 🔍 数据包验证 (无校验版本)

### 验证说明
本版本已移除校验和机制，简化数据包结构，提高传输效率。

### 验证方法
仅通过协议头和协议尾进行基本验证：

```c
bool validate_packet(const uint8_t *data, uint8_t len) {
    if (len < 20) return false;
    
    // 检查厂商ID
    if (data[2] != 0xE5 || data[3] != 0x02) return false;
    
    // 检查协议头
    if (data[4] != 0xAA || data[5] != 0x55) return false;
    
    // 检查协议尾
    if (data[19] != 0x99) return false;
    
    return true;
}
```

---

## 📡 广播参数配置

### 基站广播配置
```c
// 广播间隔
#define BLE_ADV_INTERVAL_MIN 100         // 100ms
#define BLE_ADV_INTERVAL_MAX 150         // 150ms

// 可靠性配置
#define BLE_ADV_REPEAT_COUNT 3           // 每个命令重复3次
#define BLE_ADV_REPEAT_INTERVAL 50       // 重复间隔50ms

// 广播持续时间
#define BROADCAST_DURATION_MS 1000       // 每次广播1秒
```

### 序列号管理
- **初始值**：1
- **递增方式**：每次广播自动递增
- **溢出处理**：达到65535后回到1
- **用途**：防止标签重复执行相同命令

---

## 🛠️ 标签端对接要求

### 1. BLE扫描配置
```c
esp_ble_scan_params_t scan_params = {
    .scan_type = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,  // 50ms
    .scan_window = 0x30,    // 30ms
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
};
```

### 2. 数据包识别
```c
bool is_valid_broadcast_packet(const uint8_t *data, uint8_t len) {
    if (len < 20) return false;
    
    // 检查AD类型
    if (data[1] != 0xFF) return false;
    
    // 检查厂商ID (0x02E5)
    if (data[2] != 0xE5 || data[3] != 0x02) return false;
    
    // 检查协议头
    if (data[4] != 0xAA || data[5] != 0x55) return false;
    
    // 检查协议尾
    if (data[19] != 0x99) return false;
    
    return true;
}
```

### 3. 命令解析
```c
typedef struct {
    uint8_t cmd_type;
    uint8_t group_id;
    uint16_t sequence;
    uint8_t priority;
    uint8_t params[8];
} parsed_command_t;

bool parse_broadcast_command(const uint8_t *data, parsed_command_t *cmd) {
    if (!is_valid_broadcast_packet(data, 20)) return false;
    
    cmd->cmd_type = data[6];
    cmd->group_id = data[7];
    cmd->sequence = (data[9] << 8) | data[8];  // 小端序
    cmd->priority = data[10];
    memcpy(cmd->params, &data[11], 8);
    
    return true;
}
```

---

## 🔧 调试和测试

### 1. 基站日志输出
```
I (12345) BT_BROADCAST_MGR: Started advertising packet, length: 22, duration: 1000ms
I (12345) BT_BROADCAST_MGR: Broadcasting command type=1, group=255, seq=1
I (12345) BT_BROADCAST_MGR: Broadcast completed successfully
```

### 2. 数据包调试
使用以下工具验证广播包：
- **nRF Connect** (手机APP)
- **Wireshark** (电脑抓包)
- **ESP32 BLE Scanner** (自制工具)

### 3. 常见问题排查

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 标签无响应 | 扫描参数不当 | 调整扫描间隔和窗口 |
| 数据包识别失败 | 厂商ID过滤错误 | 检查厂商ID匹配 |
| 校验和错误 | 计算范围错误 | 确认校验和计算范围 |
| 命令重复执行 | 序列号未检查 | 实现序列号去重 |

---

## 📈 性能指标

### 广播性能 (无校验版本)
- **广播间隔**：100-150ms
- **数据包大小**：21字节 (减少1字节校验和)
- **传输速率**：每秒6-10个数据包
- **覆盖范围**：室内50米，室外100米

### 可靠性指标
- **重传次数**：3次
- **成功率**：>98%（在覆盖范围内，无校验机制）
- **响应延迟**：<10ms
- **序列号防重**：支持65535个序列号
- **验证机制**：协议头尾验证

---

## 📞 技术支持

### 联系方式
- **项目负责人**：ESP32基站开发团队
- **技术文档**：参考本说明文档
- **调试工具**：提供BLE调试工具

### 更新记录
- **V2.0**：完整的广播架构实现 (无校验版本)
  - 移除校验和机制，简化数据包结构
  - 调整应用场景为快递驿站管理
  - 增加电池预警功能
  - 移除紧急疏散等警报功能
- **V1.0**：初始GATT架构（已废弃）

---

**📄 版权说明**：本文档为ESP32智能基站项目技术文档，仅供开发使用。