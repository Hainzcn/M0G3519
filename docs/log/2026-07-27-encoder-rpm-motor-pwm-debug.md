# 任务日志：编码器转速、电机 Demo 与 PWM 联调

| 字段 | 内容 |
| --- | --- |
| 起始提交 | `3e8475b99415d903719b575bf29ee4aa2edd4523` — feat: 添加每秒心跳报文与电机转速计算 |
| 涉及提交 | `5cc67ef` 编码器防溢出；`ca120d0` 电机 Demo + PWM 库修复 |
| 工作区状态 | `ca120d0` 之上尚有未提交改动（双 PWM 回退、文档补充） |
| 日期 | 2026-07-26 ~ 2026-07-27 |
| 摘要 | 完善 GM37-520 编码器配置与 RPM 换算；心跳改输出转速；编码器 16 位防溢出；电机转动 Demo；修复逐飞 PWM 库 CH2/CH3 初始化笔误；TB6612 板端联调与驱动方式选型 |

## 1. 任务背景

在 `3e8475b` 已完成 QEI 编码器接入、1 Hz 心跳报文与 RPM 计算的基础上，继续完成：

1. 编码器长时间运行溢出防护。
2. 电机转动测试 Demo 接入主循环。
3. 电机不转问题排查与 PWM 链路修复。
4. TB6612 驱动方式（双 PWM vs 符号-幅值）评估与定案。

## 2. 提交与变更概览

| 提交 | 说明 | 主要文件 |
| --- | --- | --- |
| `5cc67ef` | 编码器 16 位环形差分 + int32 累计里程 | `encoder.c/h`, `encoder_hw.c/h`, `pin.md` |
| `ca120d0` | 非阻塞电机 Demo；`heartbeat_get_ms()`；修复 `CCCTL_23` | `motor_app.c/h`, `main.c`, `zf_driver_pwm.c`, `heartbeat*.c/h`, `motor_hw.c/h` |
| 工作区未提交 | 回退双 PWM；补充 TB6612 联调文档 | `motor_hw.c/h`, `pin.md` |

## 3. 编码器（GM37-520）

### 3.1 参数与 RPM 换算

| 参数 | 值 |
| --- | --- |
| 电机型号 | GM37-520 |
| 编码器线数 | 11 线（电机轴） |
| 减速比 | 30:1 |
| QEI 倍频 | ×4（正交硬件解码） |
| 输出轴每转计数 | **1320** (= 11×4×30) |

逐飞库无现成 RPM API；中间层在心跳周期内差分计数：

`rpm = Δcount × 60000 / (1320 × period_ms)`

### 3.2 防溢出（`5cc67ef`）

| 问题 | 处理 |
| --- | --- |
| 硬件 QEI 为 16 位（LOAD=65535），约 50 圈（输出轴）回绕 | 原始计数用 `uint16` 读取 |
| `int16` 返回在 >32767 时符号解释错误 | `encoder_hw_get_raw_count()` 转 `uint16` |
| 差分跨回绕误判 | 环形差分 `(int16)(now - last)` |
| 累计里程 | `encoder_get_*_total_count()` 累加到 `int32` |

新增 API：`encoder_get_*_raw_count()`、`encoder_get_*_total_count()`、`encoder_clear_all_count()`；移除易错的 `int16 encoder_get_*_count()`。

### 3.3 硬件引脚（不变）

| 侧别 | 定时器 | A 相 | B 相 |
| --- | --- | --- | --- |
| 左轮 | TIMG8 | B10 | B11 |
| 右轮 | TIMG9 | B7 | B9 |

## 4. 心跳串口

| 项目 | 内容 |
| --- | --- |
| 周期 | 1 Hz（`HEARTBEAT_PERIOD_MS = 1000`） |
| 报文格式 | `[hb],<序号>,<左RPM>,<右RPM>\r\n` |
| 上电 | `BOOT OK\r\n` |
| 节拍 | SysTick 1 ms；新增 `heartbeat_get_ms()` 供 Demo 非阻塞定时 |

## 5. 电机转动 Demo（`motor_app`）

### 5.1 配置宏（`motor_app.c` 顶部）

| 宏 | 当前值 | 说明 |
| --- | --- | --- |
| `MOTOR_APP_DEMO_ENABLE` | 1 | 是否接入主循环 |
| `MOTOR_APP_DEMO_HOLD_FORWARD` | 1 | 1=持续正转满速（联调 VM/输出） |
| `MOTOR_APP_DEMO_DUTY` | 10000 | PWM 占空比满量程 |
| `MOTOR_APP_DEMO_STEP_MS` | 2000 | 正/反转各持续 ms（`HOLD_FORWARD=0` 时） |
| `MOTOR_APP_DEMO_LOOP` | 1 | 是否循环正反转 |

### 5.2 主程序

```c
while (1)
{
    heartbeat_app_process();
    motor_app_demo_process();
}
```

Demo 为非阻塞状态机，与心跳并行，不在中断中调用。

## 6. PWM 与 TB6612 驱动

### 6.1 逐飞库修复（`ca120d0`）

`zf_driver_pwm.c` 中 `pwm_init()` 初始化 **CH2/CH3** 时误写 `CCCTL_01`，导致：

- CH2/CH3 配置错误；
- 后续 init **覆盖** CH0/CH1 配置；
- 四路 PWM 均无有效波形 → 电机不转。

**修复**：CH2/CH3 分支改为写 `CCCTL_23`。

### 6.2 驱动方式选型

| 方式 | 正转接法 | TB6612 行为 | 结论 |
| --- | --- | --- | --- |
| **双 PWM（当前）** | IN1=PWM，IN2=低 | 高→H,L 驱动；低→L,L 滑行 | ✓ 占空比语义正确，标准接法 |
| 符号-幅值（曾短暂采用） | IN1=1，IN2=PWM | 高→**H,H 刹车**；低→H,L 驱动 | ✗ 占空比反相，不适合 TB6612 |

联调中曾改为符号-幅值以便万用表读方向脚稳态 3.3V；分析真值表后 **已回退双 PWM**（工作区未提交）。

### 6.3 当前引脚与逻辑

| 信号 | 引脚 | TIM_A0 |
| --- | --- | --- |
| 左 AIN1 / AIN2 | A0 / A1 | CH0 / CH1 |
| 右 BIN1 / BIN2 | B12 / B13 | CH2 / CH3 |

- 正转：IN1=PWM，IN2=0；反转：IN1=0，IN2=PWM。
- init 顺序：CH0 → CH1 → CH2 → CH3。

## 7. 板端联调记录

| 现象 | 结论 |
| --- | --- |
| 初测 STBY 为 0V | 需跳线拉高；应接 **3.3V（VCC）**，不可接 5V |
| STBY→3.3V 后，AIN/BIN 输入电压正确 | MCU / PWM 软件链路正常 |
| VM≈12V，输入全正确，AO/BO 输出恒 0 | **TB6612 芯片（或模块）损坏**，需更换 |
| 符号-幅值阶段 AIN1≈3.3V、AIN2≈2.2V | 输入有波形，但驱动方式不符合真值表，已回退 |

双 PWM 正转 70% 占空时万用表预期：驱动侧一路 ~2.3V（PWM 平均）、另一路 ~0V。

## 8. 验证状态

| 项目 | 状态 |
| --- | --- |
| EIDE / armclang 编译链接 | 通过 |
| UART `BOOT OK` + 1 Hz `[hb],n,Lrpm,Rrpm` | 板端已确认 |
| 编码器 QEI + RPM 心跳 | 已实现，待新电机验证 |
| PWM 四路输出（库修复 + 双 PWM） | 软件侧已修复；待换新 TB6612 复测 |
| 电机实际转动 | **阻塞于硬件**：原 TB6612 输出失效 |

## 9. 设计约束（更新）

1. **TB6612**：使用双 PWM；STBY/VCC→3.3V；VM 独立 5~12V；电机线接 AO/BO，编码器线不接 AO。
2. **PWM 库**：使用 `src/MSPM0G3519_Library` 拷贝，含 `CCCTL_23` 修复；四路 init 必须 CH0→CH3 顺序。
3. **编码器**：16 位原始计数 + 环形差分；RPM 在 1 Hz 心跳更新；总里程用 `int32` 累加。
4. **UART**：仍仅 SysConfig 初始化，禁止对 UART0 调用逐飞 `uart_init()`。

## 10. 后续计划

- [ ] 更换 TB6612 模块/芯片后复测 AO/BO 输出与电机转动。
- [ ] 确认正反转方向，必要时在 `motor_hw` 或 `motor` 层取反。
- [ ] 换片稳定后可将 `MOTOR_APP_DEMO_HOLD_FORWARD` 改回正反转交替，或关闭 Demo 进入循迹。
- [ ] 提交工作区未提交的「双 PWM 回退」与 `pin.md` 文档更新。
- [ ] 循迹传感器模块接入与闭环速度控制（`motor_set_target_rpm`）。

## 11. 相关文档

- 引脚与联调：`docs/pin/pin.md`
- API 说明：`docs/api/README.md`（部分条目仍描述 500 ms 旧心跳格式，待同步）
- 上一阶段日志：`docs/log/2026-07-26-motor-heartbeat-init.md`（基准提交 `a74c950`）
