# Emm42 步进电机驱动

本项目已按官方 MSPM0G3507 串口例程，将 Emm42 V5.0 串口协议接入 MSPM0G3519。上电仅初始化 UART 和接收中断，不使能、不回零、不转动电机。

## 接线

| MSPM0G3519 | Emm42 | 说明 |
|---|---|---|
| PA23 / UART7 TX | PLC 驱动电路 MCU 侧 RX / RAH | 3519 向接口电路发送命令 |
| PA24 / UART7 RX | PLC 驱动电路 MCU 侧 TX / TBL | 接收接口电路返回状态 |
| GND | GND | 必须共地 |

串口参数为 `115200-8-N-1`。这是 PLC 驱动接口，PA23/PA24 只能连接接口电路的 MCU 逻辑侧，必须保留原有隔离、限流和电平转换，不得绕过接口电路直连 24V PLC 端子。电机与驱动器使用独立、符合额定值的电源，不从核心板供电。首次上电前应架空机构或脱开连杆，并确认机械限位。

Emm42 独占 UART7。原 UART0（PA10/PA11）继续输出 `BOOT OK`、心跳和控制调试文本；UART1（PA8/PA9）继续用于 IMU，三路串口互不共享。

## 接口

头文件：`src/middle/emm42.h`

- `emm42_set_enabled()`：使能或去使能。
- `emm42_run_velocity()`：速度模式，带符号 RPM 表示方向。
- `emm42_move_pulses()`：位置模式，脉冲数符号表示方向。
- `emm42_move_angle()`：按默认 3200 脉冲/圈换算角度。
- `emm42_stop()`：立即停止。
- `emm42_home()`：按驱动器已保存的回零参数触发回零。
- `emm42_set_current_position_zero()`：将当前位置清零。
- `emm42_query_position()` / `emm42_query_velocity()`：请求实时位置/转速。
- `emm42_read_frame()`：在主循环中非阻塞提取一帧返回数据。
- `emm42_decode_position_deg()` / `emm42_decode_velocity_rpm()`：解码查询结果。

`EMM42_DEFAULT_PULSES_PER_REV` 必须与驱动器当前细分设置一致。官方例程采用 16 细分，即 3200 脉冲/圈；若现场设置不同，应修改该宏后再使用角度接口。

## 最小台架验证

当前固件已启用自动测试 demo：上电等待驱动器 500ms，把当前位置设为零点并使能，以 60RPM 正转两圈，再反转两圈，最后下发绝对零位命令并停止。demo 位于 `src/app/emm42_demo_app.c`，执行期间底盘电机保持停止；UART0 输出 `[emm42-demo]` 阶段日志。

以下代码应由临时测试入口或后续摆杆应用调用，不要直接设为正式固件的上电动作：

```c
emm42_set_enabled(EMM42_DEFAULT_ADDRESS, 1u, 0u);
emm42_move_angle(EMM42_DEFAULT_ADDRESS, 10.0f, 60u, 20u,
                 EMM42_POSITION_RELATIVE_TO_CURRENT, 0u);
```

验证顺序：

1. 脱开负载，确认 `emm42_set_enabled()` 返回非零且驱动器响应正常。
2. 先用 `5~10 deg`、`30~60 RPM` 验证方向；方向错误时调整角度符号或机械安装定义。
3. 调用位置查询并在主循环轮询 `emm42_read_frame()`，确认解码角度与实际方向一致。
4. 再连接摆杆，实测可用角度、速度、加速度和回零方式，之后才能接入滚球闭环。

每条发送接口会在发命令前清空旧接收数据，因此必须先取完上一条命令的返回帧，再发送下一条命令。多电机同步时，各运动命令传 `synchronized=1`，最后调用 `emm42_start_synchronized()`。

`emm42_frame_t` 首次使用时应初始化为 `{0}`。一帧读取完成后可直接复用同一对象，下一次 `emm42_read_frame()` 会自动开始接收新帧。
