# 任务日志：UART3 / 蓝牙串口测试模式

| 字段 | 内容 |
| --- | --- |
| 提交 | `01a14c7` feat: 添加蓝牙 UART 支持与测试功能 |
| 日期 | 2026-07-30 |
| 摘要 | 在 PB12/PB13 上启用 UART3，实现 RX 中断缓冲与非阻塞 TX，上电进入字节回显联调模式；底盘与 Emm42 保持停机 |

## 1. 背景

Emm42 驱动（`8929637`）接入后，需要一条独立串口与 MaixCAM2 / 蓝牙模块做双向通信验证。本提交先以**回显测试模式**确认物理层与驱动稳定性，再为后续底盘遥测（`0639022`）与视觉解析（`a026f4b`）铺路。

## 2. 系统配置

| 项 | 值 |
| --- | --- |
| 外设 | UART3（SysConfig 中初命名为 UART4 实例，引脚 PB12/PB13） |
| 波特率 | 115200-8-N-1 |
| TX | PB12 |
| RX | PB13 |
| 电平 | 3.3 V TTL |

生成文件：`M0G3519.syscfg`、`keil/ti_msp_dl_config.c/h`。

## 3. 代码变更（按模块）

### 3.1 硬件层 `src/hardware/bluetooth_hw.c/h`（后改名为 `uart3_maix_hw`）

- RX：中断 + 256 字节环形缓冲，溢出计数。
- TX：256 字节非阻塞环形缓冲 + `tx_pump()` 主循环泵送。
- 字符串发送、原子整帧写入（`write_atomic`）接口。

### 3.2 应用层 `src/app/bluetooth_test_app.c/h`（后改名为 `uart3_maix_app`）

- 上电发送启动提示。
- 1 Hz 周期性 `[bt] alive`（后续统一为 `[link] alive`）。
- 主循环将 RX 缓冲字节原样回显（**已在 `a026f4b` 废止**）。

### 3.3 主程序 `src/main.c`

- 联调阶段**仅**初始化 heartbeat + bluetooth_test；不启动 motor、IMU、OLED、grayscale、emm42。
- `control_config.h`：`MOTOR_APP_AUTO_START_LINE_FOLLOW = 0`、`MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO = 0`。

### 3.4 文档

- `docs/pin/pin.md`：补充 UART3 引脚表。
- `docs/log/2026-07-30-bluetooth-uart-test-mode.md`：接线与验收（后重命名为本文件）。

## 4. 接线

| 蓝牙 / Maix | MSPM0G3519 |
| --- | --- |
| RX | PB12 / UART3 TX |
| TX | PB13 / UART3 RX |
| GND | GND |

## 5. 验收（历史）

1. 连接后周期性收到 `[bt] alive`。
2. 发送任意字节流，应收到完全相同回显。
3. 电机、循迹、Emm42 均不应自动动作。

## 6. 后续演进

| 提交 | 变更 |
| --- | --- |
| `0639022` | 同一 UART3 增加 10 Hz 底盘遥测 V1 |
| `a026f4b` | RX 改入 24 字节视觉解析器，删除回显 |
| `41a45c7` | `bluetooth_*` 全局重命名为 `uart3_maix_*` |
| `80bd811` | 当前默认发送 100 Hz 摆杆遥测 `0x82`，见 `2026-07-31-balance-pendulum-demo-and-button.md` |

> **当前固件**：输出 `[link] alive`；UART3 RX 进入 `vision_link` 解析器，不再回显。本页保留 UART3 物理层首次接入记录。
