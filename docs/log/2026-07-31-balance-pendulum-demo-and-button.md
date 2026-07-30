# 任务日志：摆杆 ±5° Demo、摆杆遥测 V1 与按键模块

| 字段 | 内容 |
| --- | --- |
| 提交 | `80bd811` feat: 添加摆杆正负 5 度 Demo 和按钮处理功能 |
| 前置 | `8929637` Emm42 驱动；`10264aa` 遥测框架 |
| 日期 | 2026-07-31 |
| 摘要 | 实现 **连杆逆解 + Emm42 往复 Demo**；UART3 新增 **100 Hz 摆杆遥测 0x82**；新增 **4 键防抖** 与 OLED 显示 |

## 1. 运行模式（当前 HEAD 默认）

`control_config.h`：

```c
#define EMM42_BALANCE_DEMO_ENABLE              (1u)
#define UART3_MAIX_CHASSIS_TELEMETRY_ENABLE     (0u)
#define UART3_MAIX_BALANCE_TELEMETRY_ENABLE     (EMM42_BALANCE_DEMO_ENABLE)
#define MOTOR_APP_AUTO_START_LINE_FOLLOW        (0u)
```

上电：**底盘停机**，运行摆杆 Demo + 视觉 RX + 摆杆遥测 TX；底盘 0x81 与 Demo **编译互斥**。

## 2. 摆杆 Demo（`src/app/emm42_demo_app.c/h`）

### 2.1 连杆参数（实测尺寸，单位 cm）

| 符号 | 值 | 含义 |
| --- | --- | --- |
| CB | 21.0 | 曲柄长度 |
| DX, DY | 15.5, -0.5 | 机架偏置 |
| DP, BP | 3.5, 4.6 | 连杆/从动臂 |
| α | ±5° | 目标摆杆角 |
| branch | -1.0 | 曲柄右下分支（`EMM42_DEMO_LINKAGE_BRANCH`） |

### 2.2 逆解与可达性

- `emm42_demo_linkage_inverse()`：给定 α 求曲柄角 θ。
- 上电 `emm42_demo_calculate_targets()`：计算 ±5° 对应电机角（约 **+22.96° / -18.23°** 相对水平零位）。
- ρ 超出连杆可达包络 → `EMM42_DEMO_ERROR`，UART0 报 `[balance-demo] linkage target unreachable`。

### 2.3 状态机

```
WAIT_POWER(3s) → WAIT_ZERO → WAIT_ENABLE →
  MOVE_POSITIVE(+5°) → WAIT(1.5s) →
  MOVE_NEGATIVE(-5°) → WAIT(1.5s) → 循环
```

- 3 s 内人工保持摆杆水平；`emm42_set_current_position_zero()` 设零。
- 30 RPM、acc=20 位置模式绝对运动。
- 失败路径：`emm42_demo_fail()` 停机 + 错误日志。

### 2.4 主程序

- `main.c`：`#if EMM42_BALANCE_DEMO_ENABLE` 挂载 init/process。
- 与 motor/line 控制并行，**不**自动启动底盘。

## 3. 摆杆遥测 V1（`src/app/uart3_maix_app.c`）

| 项 | 值 |
| --- | --- |
| type | `0x82` |
| version | `0x01` |
| frame_len | 20 |
| 频率 | 100 Hz |
| CRC | CCITT-FALSE，0..17 |

| 字段 | 来源 |
| --- | --- |
| `demo_state` | `emm42_demo_app_get_state()` |
| `target_angle_cdeg` | Demo 目标摆杆角 ×100 |
| `imu_pitch_cdeg` | IMU 原始 pitch ×100；无效 `INT16_MIN` |
| flags | bit0 Demo 活跃；bit1 IMU 角度有效 |

**注意**：`imu_pitch` 尚未减水平零偏、未应用方向系数，仅用于机构响应曲线对比。

文档：`docs/UART3视觉通信MVP.md` §4；`docs/Emm42步进电机驱动.md` §摆杆 Demo。

## 4. 按键模块（新增）

### 硬件层 `src/hardware/button_hw.c/h`

| 按键 | 引脚 | 逻辑 |
| --- | --- | --- |
| SW1 | A30 | 内部上拉，按下=低 |
| SW2 | A31 | 同上 |
| SW3 | B1 | 同上 |
| SW4 | B0 | 同上 |

### 中间层 `src/middle/button.c/h`

- **20 ms 防抖**（`BUTTON_DEBOUNCE_MS`）。
- `button_get_active()`：返回首个稳定按下键（SW1 优先）。

### 应用层 `src/app/button_app.c/h`

- OLED **第 4 页**显示 `Key: SWx` 或 `---`。
- 脏页缓存，仅变化时刷新。

### 主程序

- `button_app_init/process` 加入主循环（与 oled_app 协同）。

## 5. 文档更新

- `docs/Emm42步进电机驱动.md`：Demo 流程、安全须知、分支/方向排查。
- `docs/UART3视觉通信MVP.md`：§4 摆杆遥测、§5 验收、当前默认模式说明。

## 6. 下游解码工具（BallBalanceRover / SBDandT）

- `SBDandT/recognition/tools/balance_telemetry.py`：解析 0x82 帧 → CSV。
- `SBDandT/recognition/tools/test_balance_telemetry.py`：单元测试。
- 录制：`uart_log_receiver.py` raw 模式。

## 7. 安全与验收

1. **首次运行**：架空机构或脱开连杆；确认急停与限位。
2. 水平零位：3 s 窗口内人工扶平；UART0 应见 `horizontal position set to zero`。
3. 往复：±5° 切换周期 ≈ 3 s（1.5 s 停留 ×2 + 运动时间）。
4. Maix/PC：100 Hz 0x82 帧 CRC 全过；`target_angle` 在 +5/-5 间切换。
5. 按键：OLED 第 4 页随 SW1–SW4 更新。

## 8. 文件变更统计

| 文件 | 说明 |
| --- | --- |
| `emm42_demo_app.c` | +208 行级重构（逆解+状态机） |
| `uart3_maix_app.c` | +77 行 balance_telemetry |
| `button_*.c/h` ×3 层 | 全新 ~220 行 |
| `control_config.h` | Demo/遥测开关 |
| `main.c` | 挂载 button + emm42 demo |
