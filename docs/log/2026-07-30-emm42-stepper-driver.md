# 任务日志：Emm42 步进电机驱动接入

日期：2026-07-30  
提交：`8929637` feat: 添加 Emm42 步进电机驱动支持

## 背景

H 题摆杆机构需要 Emm42 V5.0 步进驱动器通过 PLC 接口电路受控。本提交在 MSPM0G3519 上完成 UART7 硬件层、协议中间层和 Demo 应用骨架，为后续连杆逆解与摆杆闭环奠定基础。

## 变更摘要

| 区域 | 文件 | 说明 |
| --- | --- | --- |
| 系统配置 | `M0G3519.syscfg`、`keil/ti_msp_dl_config.*` | 新增 UART7 实例，PA23 TX / PA24 RX |
| 硬件层 | `src/hardware/emm42_hw.c/h` | UART7 中断 RX 环形缓冲、非阻塞 TX |
| 中间层 | `src/middle/emm42.c/h` | Emm42 串口协议：使能、速度/位置、回零、查询、同步 |
| 应用层 | `src/app/emm42_demo_app.c/h` | 上电初始化占位 Demo（后续提交扩展为摆杆往复） |
| 主程序 | `src/main.c` | 条件编译挂载 `emm42_demo_app` |
| 文档 | `docs/Emm42步进电机驱动.md` | 接线、接口说明、验证顺序 |
| 文档 | `docs/pin/pin.md`、`docs/api/README.md` | 补充 UART7 引脚与 API 索引 |

## 硬件与协议要点

- **串口**：UART7，115200-8-N-1，PA23→驱动接口 RX，PA24←驱动接口 TX。
- **隔离**：PA23/PA24 仅接 PLC 接口电路 MCU 逻辑侧，不得绕过隔离直连 24V 端子。
- **独占性**：Emm42 独占 UART7；UART0 调试、UART1 IMU 互不干扰。
- **默认细分**：`EMM42_DEFAULT_PULSES_PER_REV = 3200`（16 细分），须与驱动器现场设置一致。

## 中间层接口（`emm42.h`）

- 运动：`emm42_set_enabled()`、`emm42_run_velocity()`、`emm42_move_pulses()`、`emm42_move_angle()`、`emm42_stop()`、`emm42_home()`
- 标定：`emm42_set_current_position_zero()`
- 查询：`emm42_query_position()` / `emm42_query_velocity()` + `emm42_read_frame()` 非阻塞收帧
- 解码：`emm42_decode_position_deg()`、`emm42_decode_velocity_rpm()`
- 多机：`synchronized=1` 发命令后调用 `emm42_start_synchronized()`

每条发送前清空旧 RX 数据；一帧收完再发下一条，避免应答混淆。

## 后续提交依赖关系

本提交仅提供驱动能力，Demo 在 `80bd811` 才实现连杆逆解与 ±5° 往复；UART3 摆杆遥测在同期提交。当前仓库 HEAD 默认 `EMM42_BALANCE_DEMO_ENABLE=1`。
