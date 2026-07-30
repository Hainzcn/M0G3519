# 任务日志：底盘双层 PID 循迹框架与 IMU/OLED 底层重构

| 字段 | 内容 |
| --- | --- |
| 起始提交 | `11f82b854d042f3339e59503e72bdfb746b902ce` — feat: 新增 ATK-MS901M 模块用户手册及 UART 接口实现 |
| 涉及提交 | `9b4400a` UART2/DMA + I2C/OLED 重构；`5af4745` 循迹两驱 PID 框架；`2284cb8` 误差滤波与自动循迹；`06bb5b0` 电机极性/通道配置；`8b0a33c` 右弯检测与曲率前馈 |
| 当前 HEAD | `8b0a33cceedf8e4a5cac3d9f67b39181cd37520b` |
| 作者 | Hainzcn |
| 日期 | 2026-07-29 ~ 2026-07-30 |
| 摘要 | 自 ATK-MS901M IMU 协议切换为基线后，完成 IMU DMA 收包、OLED 驱动重写、灰度引脚修正与扫描序号；实现 100 Hz 循迹外环 + 双轮速度内环闭环；上电自动循迹；右弯曲率前馈与顺时针 1 m 圆 demo |

## 1. 任务背景

`11f82b8` 将 IMU 从单轴 5 字节协议切换为 **ATK-MS901M** 变长帧（`55 55 ID LEN DATA SUM`），并归档厂家手册与参考代码至 `docs/datasheet/ATK-MS901M/`。此前 `docs/log` 最新条目为 2026-07-28 OLED 接入，电机仍为开环 Demo（`motor_app_demo_process`），循迹仅完成灰度采集与 `[gs]` 调试。

本阶段目标：

1. 提升 IMU UART 收包可靠性（DMA 双缓冲）。
2. 稳定 OLED I2C 与刷新逻辑。
3. 修正灰度模块引脚与扫描一致性，为闭环控制做准备。
4. 落地 `docs/formula.md` 第 8 节所述 **100 Hz 双层 PID** 底盘控制框架。
5. 实车标定前的符号检查、遥测与调参文档。

## 2. 提交时间线

| 提交 | 日期 | 说明 |
| --- | --- | --- |
| `11f82b8` | （基线） | ATK-MS901M 协议解析；`imu_app`/`imu_hw`/`imu.c` 大改；删除旧单轴逻辑 |
| `9b4400a` | 2026-07-29 | UART1 FIFO + DMA RX；I2C 模拟故障滤波；OLED 硬件层重写；删除 `tmp/imu_parser_test.c` |
| `5af4745` | 2026-07-29 | 循迹两驱 PID 框架代码 + 文档；灰度引脚/扫描序号；`motor_app` 由 Demo 改为控制调度 |
| `2284cb8` | 2026-07-29 | 上电自动循迹；误差低通滤波；目标 RPM 斜率限制；轮速 PWM 斜率限制 |
| `06bb5b0` | 2026-07-29 | 电机输出极性、左右通道交换、编码器符号、左右 PWM 映射比例 |
| `8b0a33c` | 2026-07-30 | 右弯检测（第 6~8 路）；1 m 圆曲率前馈；顺时针圆 demo；OLED 转向指示 |

## 3. 提交 `9b4400a`：UART DMA 与 OLED/IMU 底层

### 3.1 SysConfig

| 外设 | 变更 |
| --- | --- |
| UART1（IMU，PA8/PA9） | 启用 RX FIFO、DMA 接收（`DMA_CH0`）、`DMA_DONE_RX` 中断 |
| I2C0（OLED，PB0/PB1） | 模拟故障滤波器 50 ns（原 DISABLED） |

### 3.2 IMU 硬件层

- 由 **RX 中断逐字节入 FIFO** 改为 **DMA 环形多 block**（`imu_hw_rx_blocks[][]`）。
- ISR 处理 `DL_UART_MAIN_IIDX_DMA_DONE_RX`，块状态机：`DMA → READY → 消费`；队列溢出时可恢复 DMA 通道。
- 中间层 `imu.c` 从 block 取字节喂变长帧 FSM，解析姿态（0x01）与加速度（0x03）。

### 3.3 OLED 硬件层

- `oled_hw.c` 大幅精简/重写：初始化探测、按页/按段推送、与 middle 层帧缓冲协作。
- `oled.c` 新增 `oled_refresh_pages`、页段清除、循迹条直写页缓冲等 API。
- `oled_app.c` 引入 **脏页/数据缓存** 与 **恢复周期**（模块未就绪时 1 s 重试），避免无效 I2C。

### 3.4 工程脚本

- `keil/syscfg.bat` 路径与错误处理改进；`keil/.gitignore` 忽略 build 产物。

## 4. 提交 `5af4745`：循迹两驱 PID 控制框架（核心）

### 4.1 控制结构

```text
八路灰度 → line_control（加权误差、丢线、横线）
         → 循迹 PID → 左右目标 RPM
         → wheel_speed_control（双轮 PID + kS/kV/kA 前馈）
         → motor（极性/交换）→ TB6612 PWM
         ← encoder 100 Hz 反馈
```

控制周期 **10 ms**（`CHASSIS_CONTROL_PERIOD_MS`），由 `motor_app_process()` 调度。

### 4.2 新增模块

| 文件 | 职责 |
| --- | --- |
| `control_config.h` | 周期、增益初值、限幅、车体几何、自动启动宏 |
| `control_pid.c/h` | 通用 PID + 条件积分抗饱和 |
| `line_control.c/h` | 八路权重误差、丢线保持、横线标志、差速目标 |
| `wheel_speed_control.c/h` | 双轮速度 PID、前馈、PWM 映射与斜率限制 |
| `docs/循迹两驱PID框架.md` | 架构、API、符号检查、调参顺序、遥测格式 |

### 4.3 `motor_app` 重构

**移除** 原 `motor_app_demo_process()` 及全部 `MOTOR_APP_DEMO_*` 宏。

**新增运行模式**（`motor_app_mode_enum`）：

| 模式 | 说明 |
| --- | --- |
| `DISABLED` | 停止，轮速环输出 0 |
| `SPEED_TEST` | 架空轮速标定：`motor_app_set_speed_test(l, r)` |
| `RIGHT_CIRCLE_DEMO` | 顺时针 1 m 圆：`motor_app_set_right_circle_demo(center_rpm)` |
| `LINE_FOLLOW` | 落地循迹：`motor_app_set_line_follow_enabled(1)` |

安全逻辑：

- 灰度扫描序号 **30 ms** 不更新 → 复位循迹并停车。
- 丢线超过 **200 ms** → 清空 PID 并停车。
- 主循环卡顿 **>50 ms** → 跳过一次控制并复位（使用真实 elapsed 更新编码器）。

遥测（250 ms）：`[ctl] mode,e=,n=,t=,m=,u=,s=`（后续 `8b0a33c` 扩展 `r=`、`f=`）。

### 4.4 灰度模块修正（重要）

| 项目 | 旧 | 新 | 原因 |
| --- | --- | --- | --- |
| AD2 | A17 (PA17) | **A12 (PA12)** | PA17 实测无法可靠拉低 |
| OUT | A18 (PA18) | **A13 (PA13)** | PA18 为 BOOTRST BSL _invoke 采样脚，低电平可能进 ROM BSL |
| 地址写 GPIO | `writePinsVal` | **先 clear 再 set** | 避免浮空/残留位 |
| 扫描发布 | 逐通道写 `values[]` | **work 缓冲整轮拷贝** + `scan_sequence++` | 防止 OLED/控制读到撕裂快照 |

新增 API：`grayscale_get_scan_sequence()`。

### 4.5 主程序

```c
while (1) {
    motor_watchdog_kick();
    heartbeat_hw_uart_tx_pump();
    imu_app_process();
    grayscale_app_process();
    motor_app_process();      // 原 motor_app_demo_process
    heartbeat_app_process();
    oled_app_process();
}
```

`motor_watchdog_kick()` 与 `motor.c` 内看门狗配合：非零 PWM 时若主循环停止则强制 `motor_stop()`。

### 4.6 编码器测速归属

轮速控制器独占 `encoder_update_speed(period_ms)`；1 Hz 心跳 `[hb]` 只读 RPM，不再改变测速采样基准（`heartbeat.c` 移除相关副作用）。

## 5. 提交 `2284cb8`：响应性与自动循迹

| 变更 | 说明 |
| --- | --- |
| `MOTOR_APP_AUTO_START_LINE_FOLLOW=1` | 上电取得第一帧有效灰度后自动进入循迹（无按键） |
| `LINE_ERROR_FILTER_ALPHA=0.30` | 探头切换时误差一阶低通，抑制 1000 阶跃 |
| `LINE_TARGET_SLEW_RPM_PER_S=500` | 左右目标 RPM 变化率限制 |
| `WHEEL_PWM_SLEW_DUTY_PER_S=30000` | PWM duty 变化率限制，防阶跃电压 |
| 大偏差降速 | 误差越大，`base_rpm` 从 `LINE_BASE_RPM_DEFAULT` 线性降至 `LINE_MIN_RPM_DEFAULT` |

## 6. 提交 `06bb5b0`：电机接线灵活性

`control_config.h` 与 `motor.c` 新增标定宏（实车当前值见 §9）：

| 宏 | 作用 |
| --- | --- |
| `MOTOR_OUTPUT_SWAP_LEFT_RIGHT` | PWMA/PWMB 与物理左右对调 |
| `MOTOR_LEFT/RIGHT_OUTPUT_POLARITY` | 正命令对应前进/后退取反 |
| `WHEEL_LEFT/RIGHT_ENCODER_SIGN` | 编码器 RPM 符号与前进方向一致 |
| `WHEEL_LEFT/RIGHT_PWM_MAP_SCALE` | 左右轮独立 PWM 满量程映射（右轮默认 0.92） |
| `MOTOR_RATED_MAX_RPM` / `WHEEL_*_MEASURED_MAX_RPM` | 空载最高转速记录 |

文档 `docs/循迹两驱PID框架.md` 增加 §7 必做符号检查四步流程。

## 7. 提交 `8b0a33c`：右弯检测与曲率前馈

### 7.1 右弯判定

- 监视 **模块第 6~8 路**（`values[5..7]`），任一路为黑线（`LINE_BLACK_ACTIVE_LEVEL=1`）。
- 连续 **60 ms** 且 **非横线**（黑线计数 < 6）→ `right_curve_detected=1`。
- 横线（≥6 路黑）只置 `marker_detected`，不触发右弯。

### 7.2 曲率前馈

顺时针 1 m 直径圆：$\Delta n_{ff} = n_b \cdot B / D$，当前 $B=0.18\,\text{m}$、$D=1.0\,\text{m}$。

与循迹 PID 输出相加，仍受 `LINE_TURN_RPM_LIMIT` 限制。

### 7.3 顺时针圆 Demo

```c
motor_app_set_right_circle_demo(120.0f);  // 中心 120 RPM
// 或 MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO=1 上电自动
```

左右目标：$n_L = n_c + n_c B/D$，$n_R = n_c - n_c B/D$，并 clamp 至 `WHEEL_TARGET_RPM_LIMIT`。

### 7.4 OLED

- 新增 **page 5~6** 转向区：右弯时显示「转」+ 曲率字形（16×16 点阵）。
- 遥测扩展：`r=<右侧探头数><弯道标志>,f=<前馈RPM>`。

### 7.5 误差语义分离（文档强调）

| 宏 | 值 | 用途 |
| --- | --- | --- |
| `LINE_SENSOR_ACTIVE_LEVEL` | 0 | **位置 PID** 补集误差（已完成实车调参，勿随意翻转） |
| `LINE_BLACK_ACTIVE_LEVEL` | 1 | **右弯/横线/OLED 亮块** 物理黑线电平 |

## 8. 当前配置快照（`control_config.h` @ `8b0a33c`）

| 参数 | 值 |
| --- | --- |
| 自动启动 | 循迹 ON，圆 demo OFF |
| 轮距 / 轮径 | 0.18 m / 0.065 m |
| 循迹基准 / 最低 RPM | 170 / 90 |
| 差速上限 | 60 RPM |
| 软件目标 RPM 上限 | 250 |
| 循迹 PID | Kp=0.015, Ki=0, Kd=0.00018 |
| 轮速 PID | Kp=Ki=80/8（左右相同），kS/kV/kA=0（待辨识） |
| 电机极性 | 左右均为 **-1** |
| 编码器符号 | 左 +1，右 **-1** |
| 转向符号 | `LINE_STEERING_SIGN=-1` |

## 9. 验证状态

| 项目 | 状态 |
| --- | --- |
| Keil 编译链接（35 文件，+1764/-601 行 vs 基线） | 通过 |
| IMU DMA 收包 + ATK-MS901M 解析 | 已实现，待长时稳定性确认 |
| OLED 增量刷新 + 未接模块降级 | 已实现 |
| 灰度 A12/A13 新引脚 | 已改代码与 `pin.md`，待板端复测 |
| 100 Hz 双层 PID 框架 | 已实现，增益为初值 |
| 上电自动循迹 | 已启用（`MOTOR_APP_AUTO_START_LINE_FOLLOW=1`） |
| 右弯前馈 + 圆 demo | 已实现，待赛道验证 |
| 轮速 kS/kV/kA 辨识 | **未做**（当前前馈增益为 0） |
| A 点横线停车状态机 | **未做**（仅 `marker_detected` 标志） |

## 10. 设计约束（更新）

1. **灰度引脚**：AD0/1/2/OUT = A15/A16/A12/A13；禁止再使用 PA17/PA18 作传感器。
2. **控制周期**：10 ms 由 `motor_app_process` 保证；灰度扫描非阻塞，控制器通过 `scan_sequence` 判新数据。
3. **扫描一致性**：消费者必须读 `grayscale_get_values()` 完整快照，不可在扫描中途读单通道。
4. **IMU 路径**：UART1 + DMA；禁止在 init 中长时间阻塞（见 2026-07-27 IMU 日志 §8）。
5. **OLED**：I2C 刷新仅在主循环；`oled_is_ready()` 门控。
6. **电机 Demo**：旧 `motor_app_demo_*` 已删除；标定改用 `motor_app_set_speed_test`。
7. **printf**：遥测继续使用 `%u`/`%d`，避免 microlib `%lu` 问题（见灰度日志 §6）。

## 11. 后续计划

- [ ] 按 `docs/循迹两驱PID框架.md` §7 完成架空符号检查（极性、编码器、误差方向）。
- [ ] 单轮开环辨识 kS/kV，填入 `control_config.h`。
- [ ] 架空 30/60/100 RPM 阶跃，整定轮速 PID。
- [ ] 落地直线循迹，调 `LINE_KP/KD`、滤波 α、斜率限制。
- [ ] 右弯前馈与 `LINE_RIGHT_*` 通道范围实车验证；必要时交换右弯探头索引。
- [ ] `marker_detected` 接入 A 点横线停车状态机。
- [ ] IMU 航向辅助短时丢线预测。
- [ ] 比赛前降低/关闭 `[ctl]` 文本遥测。

## 12. 相关文档

- 控制框架：`docs/循迹两驱PID框架.md`
- 动力学公式：`docs/formula.md` §8
- 引脚：`docs/pin/pin.md`（灰度 A12/A13 已更新）
- API：`docs/api/README.md`
- 上一阶段日志：`docs/log/2026-07-28-oled-display-module.md`
- IMU 基线：`11f82b8` / `docs/datasheet/ATK-MS901M/`
