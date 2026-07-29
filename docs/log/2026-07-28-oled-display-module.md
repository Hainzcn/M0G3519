# 任务日志：OLED 显示模块接入与刷新优化

| 字段 | 内容 |
| --- | --- |
| 起始提交 | `1b4e5c802d10a26e391fb67bce4cdaaef73284ff` — feat: 添加 OLED 显示模块及 I2C 支持 |
| 涉及提交 | `a187d84` — feat: 优化 OLED 显示模块，增强数据渲染与刷新逻辑 |
| 日期 | 2026-07-28 |
| 摘要 | 按 hardware → middle → app 分层接入 GME12864-49（128×64 SSD1306 I2C）；SysConfig 配置 I2C0@400 kHz；仪表盘显示 Yaw/RPM/八路循迹；增量刷新与数据缓存避免无效 I2C 推送 |

## 1. 任务背景

在电机、编码器、心跳、灰度、单轴 IMU 已接入的基础上，增加本地 OLED 仪表盘，便于无 PC 串口时观察关键运行量。厂家模块为 **4 针 I2C**（SSD1306/兼容），须使用 **硬件 I2C** 以保证 400 kHz 下整屏刷新可接受（约 25 ms）。

约束延续：主循环非阻塞；I2C 无 ACK 时须优雅降级，不影响心跳/电机/IMU。

## 2. 提交与变更概览

| 提交 | 说明 | 主要文件 |
| --- | --- | --- |
| `1b4e5c8` | OLED 三层架构 + SysConfig I2C0 | `oled_hw/middle/app`、`M0G3519.syscfg`、`main.c`、`pin.md` |
| `a187d84` | 静态标签/动态数值分离；循迹条直写页缓冲；数据变化才推送 I2C | `oled_app.c`、`oled.c/h` |

## 3. 引脚分配

| 模块信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| SCL | B0 (PB0) | I2C0_SCL，硬件 I2C |
| SDA | B1 (PB1) | I2C0_SDA，硬件 I2C |
| VCC | 3.3V | 模块亦兼容 5V（板载 LDO） |
| GND | GND | 共地 |

**PB12/PB13 无 I2C 硬件复用**，须接 B0/B1。I2C 7 位地址默认 **0x3C**（SA0 接高时改 `OLED_HW_I2C_ADDR` 为 0x3D）。

与现有外设无冲突（A0/A1 电机 PWM；A8/A9 UART1 IMU；A10/A11 UART0；B2~B5 方向；B7/B9/B10/B11 编码器）。

## 4. 软件架构

```
src/
├── hardware/oled_hw   ← SysConfig I2C0 + SSD1306 init/cmd/data
├── middle/oled        ← 1 KB 帧缓冲、6×8/8×16 字库、局部/整屏刷新
└── app/oled_app       ← 仪表盘布局 + 增量渲染 + 定时推送
```

### 4.1 显示布局（`oled_app`）

| 页 | 内容 |
| --- | --- |
| 0 | 静态标题 `MSPM0G3519` |
| 1 | `Yaw:` + 整数航向（来自 `imu_get_angle()`） |
| 3 | `L:` / `R:` + 左右 RPM（来自 `encoder_get_*_rpm()`） |
| 7 | 八路循迹条（每路 16 px 宽，`grayscale_get_values()`） |

### 4.2 刷新策略（`a187d84`）

| 区域 | 周期 | 策略 |
| --- | --- | --- |
| Yaw / RPM 数值 | 100 ms | 与缓存比较，变化时 `oled_clear_page_segment` + 重绘，仅推送 page 1~3 |
| 循迹条 | 50 ms | `memcmp` 8 路缓存，变化时 `oled_fill_page_bar` 直写页缓冲，仅推送 page 7 |

I2C 400 kHz 下整屏 1024 B 约 25 ms；单页约 3 ms。避免每轮主循环全屏刷新。

### 4.3 降级行为

`oled_hw_init()` 探测 I2C ACK；失败时 `oled_is_ready()` 返回 0，`oled_app_process()` 立即 return，不影响其他模块。

## 5. 主程序

```c
oled_app_init();   // init 末尾整屏刷新一次

while (1)
{
    heartbeat_app_process();
    motor_app_demo_process();
    grayscale_app_process();
    imu_app_process();
    oled_app_process();
}
```

## 6. 中间层新增 API（`a187d84`）

- `oled_clear_area(x, y, w, h)` — 按像素矩形清缓冲
- `oled_clear_page(page)` — 清整页
- `oled_clear_page_segment(page, x, w)` — 清页内水平段（用于数值区局部重绘）
- `oled_fill_page_bar(page, values, count, block_width)` — 循迹条直写页缓冲
- `oled_refresh_pages(page_start, page_end)` — 按页范围推送 I2C

## 7. 验证状态

| 项目 | 状态 |
| --- | --- |
| SysConfig I2C0 400 kHz + Keil 编译 | 通过 |
| 无 OLED 时主循环/心跳/IMU 正常 | 设计已支持（`oled_is_ready()` 门控） |
| 屏显 Yaw/RPM/循迹随数据变化 | 待板端确认 |
| B0/B1 接线与 0x3C 地址 | 待板端确认 |

## 8. 设计约束（更新）

1. **I2C0** 由 SysConfig 初始化；应用层通过 `oled_hw_*` 访问，不另启软件 I2C。
2. **OLED 刷新** 须在主循环完成；禁止在 ISR 中调用 `oled_refresh*`（I2C 阻塞）。
3. **循迹条** 使用页缓冲直写，避免逐像素 `oled_set_pixel` 后再整页扫描。
4. **数据缓存** 比较后再推送，降低 I2C 占用，与 IMU/灰度非阻塞原则一致。

## 9. 后续计划

- [ ] 板端确认 B0/B1 接线、0x3C/0x3D 地址与显示内容
- [ ] 可选：增加浮点 Yaw（当前 `oled_app` 截断为整数）
- [ ] 可选：循迹偏差/控制状态行
- [x] 同步 `docs/api/README.md` 增加 OLED/IMU 章节

## 10. 相关文档

- 引脚与接线：`docs/pin/pin.md`（「OLED 显示模块」章节）
- API 说明：`docs/api/README.md`
- IMU 日志：`docs/log/2026-07-27-imu-uart-module.md`
