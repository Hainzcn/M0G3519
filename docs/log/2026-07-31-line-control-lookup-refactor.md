# 任务日志：循迹查表控制框架重构

| 字段 | 内容 |
| --- | --- |
| 提交 | `0f2ec26` feat: 更新循迹控制框架与文档，增强查表功能 |
| 关联 | `8355960`（曲率前馈与联调参数） |
| 日期 | 2026-07-31 |
| 摘要 | 将循迹外环由连续加权误差改为 **8 路黑线位型查表**，引入直道/右弧相位与 smoothstep 速度过渡；遥测 `line_error` 语义改为等级×1000 |

## 1. 背景

原循迹外环基于 8 路灰度的连续加权位置误差，在胶囊形赛道（1.5 m 直道 + R=0.5 m 右弧）上难以稳定区分急弯位型。本提交按实测赛道几何，改为**离散位型查表 + 相位状态机**，并与轮速内环、UART3 遥测对齐。

## 2. 核心算法变更（`src/middle/line_control.c/h`）

### 2.1 位型查表

- 8 路灰度合成 `sensor_mask`（bit i = 通道 i 见黑线）。
- `line_pattern_table[]`：约 30 条 `{mask, level}` 映射，等级范围 **-4..+4**（负=偏左，正=偏右）。
- 未匹配 mask → `line_valid=0`，进入丢线/搜索逻辑。

### 2.2 滤波与整形

- **3 点中值滤波**（`line_filter_level`）抑制单帧毛刺。
- **逐级整形**（`line_shape_level`）：等级每次最多变化 ±1；过零时插入 **40 ms 反向保持**（`LINE_LOOKUP_REVERSE_HOLD_MS`），避免左右来回抖动。

### 2.3 相位与速度

- 新增 `line_track_phase_t`：`STRAIGHT` / `RIGHT_ARC`，由编码器累计里程切换。
- 直道基准速度 `LINE_LOOKUP_STRAIGHT_BASE_RPM=170`，以 120 RPM 为反馈标定基准（`LINE_LOOKUP_SPEED_REFERENCE_RPM`）。
- 直道↔圆弧过渡区（±0.15 m）用 **smoothstep** 混合 `curve_blend`，平滑基准 RPM 与转向量。
- 四级转向 RPM：8 / 16 / 28 / 42（× `feedback_scale`，上限 55 RPM）。

### 2.4 丢线与搜索

- 无效读数 **80 ms** 内保持上一有效命令（`LINE_LOOKUP_LOST_HOLD_MS`）。
- 超过 **250 ms** 进入搜索：固定转向 RPM 35，直至重新捕获线。

### 2.5 输出结构扩展

`line_control_output_t` 新增/debug 字段：`lookup_level`、`sensor_mask`、`phase`、`phase_distance_m`、`curve_blend`、`lookup_correction_rpm`、`speed_scale`、`feedback_scale` 等。

## 3. 配置变更（`src/middle/control_config.h`）

删除旧连续误差/PID 外环参数，新增查表专用宏：

| 宏 | 典型值 | 含义 |
| --- | --- | --- |
| `LINE_BLACK_ACTIVE_LEVEL` | 1 | 见黑为 1 |
| `LINE_LOOKUP_STRAIGHT_LENGTH_M` | 1.5 | 直道长度 |
| `LINE_LOOKUP_ARC_RADIUS_M` | 0.5 | 圆弧半径 |
| `LINE_LOOKUP_ARC_LENGTH_M` | π/2×R | 弧长 |
| `LINE_LOOKUP_INITIAL_PHASE` | 0 | 上电初始相位（直道） |
| `LINE_LOOKUP_STRAIGHT_BASE_RPM` | 170 | 直道目标 |
| `LINE_LOOKUP_LEVEL_*_TURN_RPM` | 8/16/28/42 | 各级差速 |

## 4. 其他代码

- `src/app/motor_app.c`：适配新外环输出字段。
- `docs/UART3底盘遥测协议.md`：`line_error` 改为 **查表等级×1000**（-4000..4000）。
- `docs/循迹两驱PID框架.md`：重写外环章节，描述查表+相位逻辑。
- `docs/api/README.md`：更新 `line_control_*` API 说明。

## 5. 与遥测的关系

- V1 遥测（52 B）中 `line_error` 字段语义在本提交后变更。
- V2 遥测（`10264aa`，56 B）文档同步更新等级编码说明。

## 6. 验收要点

1. 直道中线：`|lookup_level| ≤ 1`，左右 RPM 差稳定。
2. 右弧入口：相位切换后 `curve_blend` 在过渡区内 0→1 单调。
3. 故意遮线：80 ms 内不失控；250 ms 后进入搜索转向。
4. OLED/调试输出中 mask 与 level 与实物位置一致。

## 7. 文件变更统计

| 文件 | 变更量 |
| --- | --- |
| `line_control.c` | +598/-258（重构主体） |
| `control_config.h` | 查表参数集 |
| `line_control.h` | 输出结构与相位枚举 |
| `motor_app.c` | 小幅适配 |
| 文档 ×3 | 协议与框架说明 |
