# 任务日志：红外循迹 I2C 重构

| 字段 | 内容 |
| --- | --- |
| 提交 | `e9d22af` feat: 添加红外循迹模块与I2C支持 |
| 日期 | 2026-07-31 |
| 摘要 | 灰度模块由 **软件 I2C / 模拟时序** 改为 **硬件 I2C1 @400 kHz**；新增通用 `i2c_bus` 事务管理器，OLED(I2C0) 与循迹(I2C1) 独立 ISR 并行 |

## 1. 背景与方案选型

8 路红外循迹模块支持 I2C 与 UART 两种接口。对比后选用 I2C（详见 `docs/红外循迹模块适配.md`）：

| 对比 | I2C | UART ASCII |
| --- | --- | --- |
| 单次读 | 1 字节 @0x30 | ~43 字节文本帧 |
| 线上时间 | ~0.1 ms | ~3.7 ms @115200 |
| CPU | 无解析 | 找帧+atoi |
| 引脚 | PA15/PA16 独占 I2C1 | 需额外 UART |

## 2. 系统配置

| 外设 | 引脚 | 速率 | 用途 |
| --- | --- | --- | --- |
| I2C0 | PB17 SCL / PB18 SDA | 400 kHz | OLED |
| I2C1 | PA15 SCL / PA16 SDA | 400 kHz | 红外循迹 |

- 模块 7 位地址：`0x12`
- 数字量寄存器：`0x30`；X1=bit7 … X8=bit0（模块输出 active-low，中间层取反）
- `M0G3519.syscfg` + `ti_msp_dl_config.*`  regenerated

**引脚冲突检查**：I2C1 与 UART3(PB12/13)、Emm42 UART7(PA23/24)、IMU UART1 无重叠。

## 3. 新增 `src/hardware/i2c_bus.c/h`（~297 行）

通用非阻塞 I2C 控制器事务：

- 双客户端：`I2C_BUS_CLIENT_OLED`、`I2C_BUS_CLIENT_IR_TRACKING`
- 状态机：IDLE → TX → RX → COMPLETE / ERROR
- ISR 处理 TX/RX FIFO、NACK、仲裁丢失
- **10 ms 超时**自动释放总线
- **I2C1（循迹）中断优先级高于 I2C0（OLED）**，避免 OLED 长写阻塞循迹 2 ms 轮询

API：`i2c_bus_init()`、`i2c_bus_write()`、`i2c_bus_write_read()`、`i2c_bus_is_busy()`。

## 4. 循迹链路改造

### `src/hardware/grayscale_hw.c/h`

- 删除 bit-bang；改为 `i2c_bus_write_read()` 读 0x30。
- 非阻塞：`start_read()` + `take_read()` 分离请求与完成。

### `src/middle/grayscale.c/h`

- **2 ms 轮询**发起读请求（`GRAYSCALE_POLL_PERIOD_MS`）。
- 保留 `grayscale_get_values()` / `grayscale_get_scan_sequence()` 对外 API **不变**。
- 新增 `grayscale_get_raw_bits()` 供 OLED 调试。
- 20 ms 无更新 → 离线判定（`GRAYSCALE_ONLINE_TIMEOUT_MS`）。

### `src/app/grayscale_app.c`

- 适配新在线/错误语义。

## 5. OLED 链路同步改造

### `src/hardware/oled_hw.c/h`

- 迁移至 `i2c_bus` 客户端 I2C0；删除内联 I2C 状态机（**-155 行**精简）。
- 与循迹共享同一套超时/错误恢复逻辑。

### `src/app/oled_app.c`

- 第 4 页显示红外原始位、I2C 错误计数、离线状态。
- 模块未就绪时保留 1 s 重试逻辑。

## 6. 控制层兼容

- `line_control.c` **无需修改**（仍消费 `grayscale_get_values()`）。
- 控制层原有 **30 ms 数据新鲜度保护** + 本提交 **10 ms I2C 超时** 共同保证丢线停车。

## 7. 文档

- 新增 `docs/红外循迹模块适配.md`
- 更新 `docs/pin/pin.md`、`docs/api/README.md`

## 8. 验收要点

1. 2 ms 周期下 `grayscale_get_scan_sequence()` 稳定递增。
2. 故意拉低 SDA：10 ms 内总线释放，OLED/循迹均恢复。
3. 循迹运行时 OLED 刷新不阻塞 I2C1 读（示波器或 `[gs]` 序列号验证）。
4. 上电**不**自动写校准寄存器 0x01（避免覆盖模块标定）。

## 9. 文件变更统计

| 文件 | 说明 |
| --- | --- |
| `i2c_bus.c/h` | +297 新增 |
| `grayscale_hw.c`、`grayscale.c` | I2C 读路径 |
| `oled_hw.c` | -155 行，迁 i2c_bus |
| `grayscale_app.c`、`oled_app.c` | 显示/在线 |
| SysConfig + ti_msp_dl_config | I2C1 实例 |
