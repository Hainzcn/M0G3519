# 2026-08-01 球位置-速度串级控制重构

## 背景

历史提交 `c206408` 开始在视觉恢复和 SW1 序列中使用球轨迹规划器，能够产生梯形
位置、速度和加速度参考。但是底层控制律仍为
`Kp * position_error + Kd * velocity_error`，其中位置误差隐式生成的速度没有限幅。
因此 V1 大扰动时，实际球速可以远高于 `BALANCE_PROFILE_MAX_VELOCITY_MPS`。

## 新控制律

控制器改为显式串级结构：

```text
velocity_command = reference_velocity +
                   position_gain * position_error
velocity_command = clamp(velocity_command, +/-max_ball_velocity)
desired_accel = reference_accel_gain * reference_accel +
                velocity_gain * (velocity_command - estimated_velocity)
```

V1 居中和 SW1 轨迹共用同一控制器。SW1 提供非零轨迹参考；V1 保持零位置参考，
由位置外环生成回中速度。边缘恢复、加速度限幅、摆角限幅和摆杆轨迹限制保持不变。

初始参数保持原 `Kp=3.0, Kd=1.5` 在中心附近的等效小信号特性：

```text
BALANCE_POSITION_LOOP_GAIN_S_INV = 2.0
BALANCE_VELOCITY_LOOP_GAIN_S_INV = 1.5
BALANCE_MAX_BALL_VELOCITY_MPS    = 0.060
```

未限幅时，等效参数仍为 `Kp = 2.0 * 1.5 = 3.0`、`Kd = 1.5`；大位置误差时
速度命令被限制为 `60mm/s`。`BALANCE_CONTROL_FLAG_VELOCITY_SATURATED` 表示速度
命令正在限幅。

## 实车调参顺序

1. 保持位置增益 `2.0`，只调速度增益，使实际速度跟随命令且制动后不反向发散。
2. 保持速度增益不变，再降低位置增益处理目标附近的往复摆动。
3. 最后调整最大速度、轨迹前视时间和最大加速度；不要同时改变三者。
4. 使用 `+/-40mm` 和 `+/-70mm` 释放测试，避开 `100mm` 边缘恢复区。

## 低速静差与 SW1 序列修正

`session_bc1b1b9c_00` 中 SW1 正向段停在约 `+37~39mm`，未进入 `+50mm`
到达容差。原状态机在正向段 `4.8s` 超时后直接把目标设为零，因此观测上只在正向
平台和零点之间运动。现改为每段独立计时：正向段到达或超时均切换到 `-50mm`，只有
负向段超时才取消序列并回零。

V1 和 SW1 平台处所需纠偏加速度约为 `0.05~0.07m/s2`，纯比例速度环必须保留较大
位置误差才能克服滚动和静摩擦。新增保守的低速摩擦补偿：

```text
BALANCE_LOW_SPEED_FRICTION_ACCEL_MPS2 = 0.045
BALANCE_CENTER_CAPTURE_POSITION_M     = 0.004
BALANCE_LOW_SPEED_THRESHOLD_MPS       = 0.010
```

补偿仅在参考轨迹已经进入 `HOLD`、位置误差大于 `4mm`、估计速度低于 `10mm/s`
时生效。运动轨迹段、中心捕获区和正常速度运动均不启用，最终输出仍受
`BALANCE_MAX_BALL_ACCEL_MPS2` 限制。
