#ifndef CONTROL_CONFIG_H_
#define CONTROL_CONFIG_H_

/* 运行模式 */
#define CHASSIS_CONTROL_PERIOD_MS                    (10u)    /* 底盘控制周期，单位：毫秒 */
#define MOTOR_APP_AUTO_START_LINE_FOLLOW             (0u)     /* 上电自动启动循迹：0=关闭，1=开启 */
#define MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO       (0u)     /* 上电自动启动右转圆周演示：0=关闭，1=开启 */

/* 菜单赛道模式 1：无负载循迹一圈并停回 A 点 */
#define NO_LOAD_LAP_CRUISE_RPM                        (170.0f) /* 正常循迹基础轮速，单位：RPM */
#define NO_LOAD_LAP_MARKER_MIN_DISTANCE_M             (5.0f)   /* 超过该里程后才允许识别启停线，单位：米 */
#define NO_LOAD_LAP_MARKER_MIN_ACTIVE_COUNT            (5u)     /* 模式1启停线至少同时覆盖的探头数 */
#define NO_LOAD_LAP_POST_MARKER_DISTANCE_M            (0.300f) /* 启停线后继续循迹的制动触发距离，单位：米 */
#define NO_LOAD_LAP_POST_MARKER_MIN_RPM                (25.0f)  /* 到达停车点前保持循迹的最低轮速 */
#define NO_LOAD_LAP_LINE_LOSS_TIMEOUT_MS              (500u)   /* 持续丢线后终止任务，单位：毫秒 */
#define NO_LOAD_LAP_SENSOR_OFFLINE_TIMEOUT_MS         (500u)   /* 灰度持续离线后终止任务，单位：毫秒 */
#define NO_LOAD_LAP_TIMEOUT_MS                        (20000u) /* 一圈任务硬超时，单位：毫秒 */

/* 菜单赛道 3/4/5：独立循迹速度 */
#define TRACK_MODE_3_LINE_FOLLOW_RPM                   (60.0f) /* 赛道 3：A-B RUN */
#define TRACK_MODE_4_LINE_FOLLOW_RPM                   (60.0f) /* 赛道 4：BALL LAP */
#define TRACK_MODE_5_LINE_FOLLOW_RPM                   (60.0f) /* 赛道 5：ANY POSITION */

/* 赛道状态 2 / 赛题第 3 项：静止小车上小球 O -> +5 cm -> -5 cm */
#define STOP_TEST_POSITIVE_TARGET_M                    (0.050f) /* 正端目标位置 */
#define STOP_TEST_NEGATIVE_TARGET_M                   (-0.050f) /* 负端目标位置 */
#define STOP_TEST_MAX_ENDPOINT_ERROR_M                 (0.010f) /* 正负端点最大允许误差 */
#define STOP_TEST_ARRIVAL_TOLERANCE_M                  (0.010f) /* 状态机端点到位阈值 */
#define STOP_TEST_VELOCITY_TOLERANCE_MPS               (0.030f) /* 到位时允许的最大球速 */
#define STOP_TEST_ENDPOINT_SETTLE_MS                   (200u)   /* 端点连续稳定确认时间 */
#define STOP_TEST_TIMEOUT_MS                           (5000u)  /* 运动全流程最大时间 */
#define STOP_TEST_READY_TIMEOUT_MS                     (10000u) /* 等待平衡控制器就绪超时 */
#define STOP_TEST_CENTER_TOLERANCE_M                   (0.005f) /* SW4 回中完成阈值 */
#define STOP_TEST_CENTER_RETRY_MS                      (100u)   /* 回中目标拒绝后的重试周期 */

/* STOP_TEST 独立滚球控制参数 */
#define STOP_TEST_POSITION_KP_S_INV                    (1.50f)
#define STOP_TEST_POSITION_KI_S2_INV                   (0.50f)
#define STOP_TEST_MAX_TARGET_VELOCITY_MPS              (0.100f)
#define STOP_TEST_BRAKING_ENVELOPE_MPS2                (0.25f)
#define STOP_TEST_VELOCITY_KV_DEG_PER_MM               (0.020f)
#define STOP_TEST_NEAR_POSITION_M                      (0.060f)
#define STOP_TEST_NEAR_GAIN                            (0.40f)
#define STOP_TEST_NEAR_SCALE_MAX                       (1.40f)
#define STOP_TEST_MAX_BEAM_ANGLE_DEG                   (4.0f)
#define STOP_TEST_MAX_BEAM_VELOCITY_DEG_S              (80.0f)

/* 赛道模式 3：A 点到 B 点，钢球保持中心 */
#define AB_RUN_FORCE_STRAIGHT_DISTANCE_M               (1.30f)  /* 此里程后忽略循迹并等速直行 */
#define AB_RUN_IGNORE_RIGHT_SHIFT_DISTANCE_M           (1.50f)  /* 右移黑线干扰起始里程；已包含在直行段内 */
#define AB_RUN_TARGET_DISTANCE_M                       (1.75f)  /* 本阶段结束里程，单位：米 */
#define AB_RUN_TIMEOUT_MS                              (8000u)  /* A 到 B 最大允许时间，单位：毫秒 */
#define AB_RUN_LINE_LOSS_TIMEOUT_MS                    (300u)   /* 连续丢线故障时间，单位：毫秒 */
#define AB_RUN_IMU_LOSS_TIMEOUT_MS                     (100u)   /* 连续无新鲜 IMU 的故障时间 */
#define AB_RUN_MAX_ERROR_M                             (0.010f) /* 赛题钢球最大允许中心误差 */

#define UART3_MAIX_MODE_NORMAL                        (0u)     /* UART3 正常模式：接收视觉数据并发送运行遥测 */
#define UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG       (1u)     /* UART3 底盘遥测调试模式 */
#define UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG       (2u)     /* UART3 摆杆遥测调试模式 */
#define UART3_MAIX_MODE                               (UART3_MAIX_MODE_NORMAL) /* UART3 当前工作模式 */

/* 视觉绝对位置校准：校正位置 = 视觉下发位置 + 本偏置 */
#ifndef BALANCE_VISION_POSITION_OFFSET_M
#define BALANCE_VISION_POSITION_OFFSET_M               (0.000f) /* 单位：米；正值将坐标向正方向平移 */
#endif

#ifndef BALL_RETURN_DEMO_ENABLE
#define BALL_RETURN_DEMO_ENABLE                       (0u)     /* 开环回球演示：0=关闭，1=开启 */
#endif
#ifndef EMM42_BALANCE_DEMO_ENABLE
#define EMM42_BALANCE_DEMO_ENABLE                     (0u)     /* EMM42 标定演示：0=关闭，1=开启 */
#endif
#ifndef BALANCE_CONTROL_ENABLE
#define BALANCE_CONTROL_ENABLE                        (0u)     /* 旧版轨迹控制器：0=关闭，1=开启 */
#endif
#ifndef BALANCE_SIMPLE_CONTROL_ENABLE
#if ((BALANCE_CONTROL_ENABLE == 0u) && \
     (EMM42_BALANCE_DEMO_ENABLE == 0u) && \
     (BALL_RETURN_DEMO_ENABLE == 0u))
#define BALANCE_SIMPLE_CONTROL_ENABLE                 (1u)     /* 简化速度控制器：其他摆杆模式关闭时默认开启 */
#else
#define BALANCE_SIMPLE_CONTROL_ENABLE                 (0u)     /* 简化速度控制器：存在互斥模式时关闭 */
#endif
#endif
#ifndef BALANCE_DRIVE_DEMO_ENABLE
#define BALANCE_DRIVE_DEMO_ENABLE                     \
    ((BALANCE_CONTROL_ENABLE != 0u) || \
     (BALANCE_SIMPLE_CONTROL_ENABLE != 0u))           /* 带球循迹一圈：支持两套摆杆控制器 */
#endif

/* 摆杆执行器通用上电参数 */
#ifndef BALANCE_STARTUP_CALIBRATED
#define BALANCE_STARTUP_CALIBRATED                    (1u)     /* 机械零位已标定：0=禁止驱动，1=允许驱动 */
#endif
#define BALANCE_POWER_WAIT_MS                          (3000u)  /* 驱动器上电等待时间，单位：毫秒 */
#define BALANCE_LOWER_STOP_SETTLE_MS                   (1500u)  /* 下机械限位稳定时间，单位：毫秒 */
#define BALANCE_LEVEL_SETTLE_MS                        (200u)   /* 回位角稳定时间，单位：毫秒 */
#define BALANCE_LEVEL_RETURN_RPM                       (20u)    /* 上电回位水平速度，单位：RPM */
#define BALANCE_EMM42_MOVE_RPM                         (120u)   /* EMM42 位置模式移动速度，单位：RPM */
#define BALANCE_EMM42_ACCELERATION                     (50u)    /* EMM42 位置模式加速度参数 */
#define BALANCE_LEVEL_MOTOR_TOLERANCE_DEG              (1.0f)   /* 上电回位允许误差，单位：度 */

/* 当前简化速度控制器：任务与观测器 */
#define BALANCE_SIMPLE_CONTROL_PERIOD_MS               (20u)    /* 控制周期，单位：毫秒 */
#define BALANCE_SIMPLE_OBSERVER_ALPHA                  (0.85f)   /* 位置残差校正系数 */
#define BALANCE_SIMPLE_OBSERVER_BETA                   (0.15f)   /* 速度残差校正系数 */
#define BALANCE_SIMPLE_VISIBLE_LIMIT_M                 (0.130f)  /* 视觉可接受位置范围，单位：米 */
#define BALANCE_SIMPLE_MAX_IMPLIED_SPEED_MPS           (0.80f)   /* 相邻视觉帧允许的最大推算速度，单位：米每秒 */
#define BALANCE_SIMPLE_MAX_CAPTURE_INTERVAL_MS         (150u)     /* 相邻视觉采样最大间隔，单位：毫秒 */
#define BALANCE_SIMPLE_VISION_TIMEOUT_MS               (250u)     /* 视觉观测超时时间，单位：毫秒 */
#define BALANCE_SIMPLE_VISION_TRANSPORT_MS             (3u)      /* 视觉链路固定传输延迟，单位：毫秒 */
#define BALANCE_SIMPLE_MAX_PREDICTION_MS                (50u)     /* 陈旧视觉状态最大外推时间，单位：毫秒 */
#define BALANCE_SIMPLE_RECOVERY_FRAMES                 (2u)      /* 观测恢复所需连续有效帧数 */
#define BALANCE_SIMPLE_MIN_CONFIDENCE                  (50u)     /* 视觉测量最低置信度 */

/* 当前简化速度控制器：位置环与速度环 */
#define BALANCE_SIMPLE_POSITION_ON_M                   (0.002f)  /* 启用位置回中控制的误差阈值，单位：米 */
#define BALANCE_SIMPLE_POSITION_OFF_M                  (0.001f)  /* 关闭位置回中控制的误差阈值，单位：米 */
#define BALANCE_SIMPLE_POSITION_KP_S_INV               (1.00f)   /* 位置比例增益，单位：每秒 */
#define BALANCE_SIMPLE_POSITION_KI_S2_INV              (0.30f)   /* 位置积分增益，单位：每平方秒 */
#define BALANCE_SIMPLE_MAX_TARGET_VELOCITY_MPS         (0.05f)  /* 最大目标球速，单位：米每秒 */
#define BALANCE_SIMPLE_BRAKING_ENVELOPE_MPS2           (0.10f) /* 回中制动包络加速度，单位：米每平方秒 */
#define BALANCE_SIMPLE_ACTUATOR_DELAY_S                (0.25f)   /* 含目标摆角爬升的等效执行延迟，单位：秒 */
#define BALANCE_SIMPLE_VELOCITY_KV_DEG_PER_MM          (0.020f)  /* 球速误差到目标摆角的增益，单位：度/(毫米/秒) */
#define BALANCE_SIMPLE_MAX_TARGET_BEAM_ANGLE_DEG       (5.0f)    /* 含车辆前馈的目标摆角绝对值上限 */
#define BALANCE_SIMPLE_TARGET_BEAM_ANGLE_SLEW_DEG_S    (30.0f)   /* 跟随底盘加速度包络，单位：度每秒 */
#define BALANCE_SIMPLE_BEAM_ANGLE_KP_S_INV             (12.0f)   /* 提高前馈姿态跟随带宽，单位：每秒 */
#define BALANCE_SIMPLE_BEAM_ANGLE_DEADBAND_DEG         (0.05f)   /* 抑制整数 RPM 往返动作的摆角死区 */
#define BALANCE_SIMPLE_MAX_BEAM_VELOCITY_DEG_S         (30.0f)   /* 最大摆杆角速度，单位：度每秒 */
#define BALANCE_SIMPLE_NEAR_POSITION_M                 (0.012f)  /* 近中心增益区间，单位：米 */
#define BALANCE_SIMPLE_NEAR_GAIN                       (0.15f)    /* 近中心附加增益，0 表示关闭 */
#define BALANCE_SIMPLE_NEAR_SCALE_MAX                  (1.15f)   /* 近中心增益最大倍率 */
#define BALANCE_SIMPLE_INTEGRAL_ZONE_M                 (0.050f)  /* 位置积分生效范围，单位：米 */
#define BALANCE_SIMPLE_INTEGRAL_LIMIT_MPS              (0.010f)  /* 位置积分对应目标球速上限，单位：米每秒 */
#define BALANCE_SIMPLE_FIXED_BEAM_BIAS_DEG             (0.2f)   /* 球状态到目标摆角的固定正向偏置，单位：度 */
#define BALANCE_SIMPLE_ACCELERATION_KA                 (0.0f)    /* 球加速度补偿增益，0 表示关闭 */
#define BALANCE_SIMPLE_ACCELERATION_FILTER_ALPHA       (0.20f)   /* 球加速度低通滤波系数 */
#define BALANCE_SIMPLE_CAR_ACCEL_OFFSET_MPS2           (0.200f)  /* IMU X 轴静态零偏 */
#define BALANCE_SIMPLE_CAR_ACCEL_GAIN                  (1.0f)    /* IMU 纵向加速度标定增益 */
#define BALANCE_SIMPLE_CAR_ACCEL_SIGN                  (1.0f)    /* IMU X 轴与车头方向关系 */
#define BALANCE_SIMPLE_CAR_ACCEL_LIMIT_MPS2            (2.0f)    /* 前馈加速度安全限幅 */
#define BALANCE_SIMPLE_CAR_IMU_MAX_AGE_MS              (25u)     /* IMU 加速度最大有效年龄 */
#define BALANCE_SIMPLE_CAR_ACCEL_FILTER_ALPHA          (0.35f)   /* 低延迟 IMU 加速度一阶低通系数 */
#define BALANCE_SIMPLE_CAR_FF_ENTER_MPS2               (0.02f)   /* 尽早跟随启动加速度包络 */
#define BALANCE_SIMPLE_CAR_FF_EXIT_MPS2                (0.01f)   /* 前馈退出加速度阈值 */
#define BALANCE_SIMPLE_CAR_FF_ENTER_MS                 (10u)     /* 一个控制周期内启用预测前馈 */
#define BALANCE_SIMPLE_CAR_FF_EXIT_MS                  (150u)    /* 前馈退出确认时间 */
#define BALANCE_SIMPLE_CAR_FF_GAIN                     (1.0f)    /* 完整惯性补偿：摆角 = atan(a/g) */
#define BALANCE_SIMPLE_CAR_FF_MAX_ANGLE_DEG            (4.0f)    /* 覆盖 0.613 m/s^2 启动所需的 3.57 度 */
#define BALANCE_SIMPLE_CAR_FF_POSITION_CUTOFF_M        (0.010f)  /* 偏差到 1 cm 时前馈完全让权给回中环 */
#define BALANCE_SIMPLE_CAR_FF_PREACTUATION_MS          (140u)    /* 启动前从零推进同长度的前馈时间轴 */
#define BALANCE_SIMPLE_CAR_FF_PREVIEW_S                (0.001f * (float)BALANCE_SIMPLE_CAR_FF_PREACTUATION_MS)
#define BALANCE_SIMPLE_CAR_FF_IMU_CORRECTION_GAIN      (0.50f)   /* IMU 对当前规划加速度残差的修正增益 */

/* 赛题 MODE_5：带球顺时针循迹一圈并通过 A 点 */
#define BALANCE_DRIVE_DEMO_APPROACH_RPM                (75.0f)   /* 接近 A 点时的基础轮速 */
#define BALANCE_DRIVE_DEMO_APPROACH_DISTANCE_M         (5.6f)    /* 进站减速里程 */
#define BALANCE_DRIVE_DEMO_BRAKE_HOLD_MS               (600u)    /* 过 A 后保持前馈覆盖制动 */
#define BALANCE_DRIVE_DEMO_MAX_ERROR_M                 (0.010f)  /* 全程最大允许中心误差 */

/* 当前简化速度控制器：执行器 */
#define BALANCE_SIMPLE_MOTOR_SIGN                      (-1.0f)  /* 摆杆正向角速度对应电机位置减小 */
#define BALANCE_SIMPLE_TRANSMISSION_RATIO              (3.80f)  /* 摆杆与电机的标称传动比 */
#define BALANCE_SIMPLE_RAISING_RATIO                   (3.80f)  /* 抬升方向传动比 */
#define BALANCE_SIMPLE_LOWERING_RATIO                  (3.80f)  /* 下压方向传动比 */
#define BALANCE_SIMPLE_MOTOR_MIN_HARD_DEG              (-35.0f) /* 电机最小硬限位，单位：度 */
#define BALANCE_SIMPLE_MOTOR_MAX_HARD_DEG              (0.0f)   /* 电机最大硬限位，单位：度 */
#define BALANCE_SIMPLE_MOTOR_MIN_SOFT_DEG              (-30.0f) /* 电机最小软限位，单位：度 */
#define BALANCE_SIMPLE_MOTOR_MAX_SOFT_DEG              (-10.0f)   /* 电机最大软限位，单位：度 */
#define BALANCE_SIMPLE_MAX_MOTOR_RPM                   (60)     /* 电机速度命令绝对值上限，单位：RPM */
#define BALANCE_SIMPLE_MIN_ACTIVE_RPM                  (1)      /* 非零电机速度命令最小值，单位：RPM */
#define BALANCE_SIMPLE_EMM42_ACCELERATION              (0u)    /* EMM42 速度模式加速度参数 */
#define BALANCE_SIMPLE_POSITION_QUERY_MS               (20u)     /* 摆角闭环位置查询周期，单位：毫秒 */
#define BALANCE_SIMPLE_VELOCITY_QUERY_MS               (100u)    /* 电机实际速度诊断查询周期，单位：毫秒 */
#define BALANCE_SIMPLE_VELOCITY_KEEPALIVE_MS           (100u)   /* 电机速度命令保活周期，单位：毫秒 */
#define BALANCE_SIMPLE_COMMAND_TIMEOUT_MS              (50u)    /* 电机命令应答超时，单位：毫秒 */
#define BALANCE_SIMPLE_MOTOR_POSITION_MAX_AGE_MS       (80u)    /* 电机位置反馈最大有效年龄，单位：毫秒 */
#define BALANCE_SIMPLE_MOTOR_VELOCITY_MAX_AGE_MS       (100u)   /* 电机速度反馈最大有效年龄，单位：毫秒 */
#ifndef BALANCE_SIMPLE_STARTUP_ACK_FALLBACK_ENABLE
#define BALANCE_SIMPLE_STARTUP_ACK_FALLBACK_ENABLE     (1u)     /* 启动阶段无应答时按定时流程继续：兼容单向 UART 接口 */
#endif
#define BALANCE_SIMPLE_STARTUP_OPEN_LOOP_LEVEL_MS      (1000u)  /* 无位置反馈时等待水平移动完成的时间，单位：毫秒 */
#define BALANCE_SIMPLE_RUNTIME_MOTOR_SAFETY_ENABLE     (0u)     /* 命令应答错误锁存：0=调试关闭，1=开启 */
#define BALANCE_SIMPLE_MAX_COMMAND_ERRORS              (3u)     /* 连续电机命令错误上限 */
#define BALANCE_SIMPLE_STARTUP_TIMEOUT_MS              (10000u) /* 执行器上电流程总超时，单位：毫秒 */

/* 当前简化速度控制器：目标与安全策略 */
#define BALANCE_SIMPLE_TARGET_LIMIT_M                  (0.090f) /* 外部目标位置绝对值上限，单位：米 */
#define BALANCE_SIMPLE_BALL_SOFT_EDGE_M                (0.100f) /* 小球软边界位置，单位：米 */
#define BALANCE_SIMPLE_BALL_HARD_EDGE_M                (0.125f) /* 小球硬边界位置，单位：米 */
#define BALANCE_SIMPLE_SAFE_RETURN_DELAY_MS            (250u)   /* 进入安全回位前的等待时间，单位：毫秒 */
#define BALANCE_SIMPLE_SAFE_RETURN_DEG_S               (6.0f)   /* 安全回位摆杆角速度，单位：度每秒 */
#define BALANCE_SIMPLE_STATIC_LOCK_ENABLE              (0u)     /* 中心静止锁定：0=关闭，1=开启 */
#define BALANCE_SIMPLE_STATIC_ENTER_POSITION_M         (0.002f) /* 进入静止锁定的位置阈值，单位：米 */
#define BALANCE_SIMPLE_STATIC_ENTER_VELOCITY_MPS       (0.003f) /* 进入静止锁定的速度阈值，单位：米每秒 */
#define BALANCE_SIMPLE_STATIC_ENTER_MS                 (500u)   /* 进入静止锁定所需稳定时间，单位：毫秒 */
#define BALANCE_SIMPLE_STATIC_RELEASE_POSITION_M       (0.004f) /* 退出静止锁定的位置阈值，单位：米 */
#define BALANCE_SIMPLE_STATIC_RELEASE_VELOCITY_MPS     (0.006f) /* 退出静止锁定的速度阈值，单位：米每秒 */

/* 底盘几何与电机接线 */
#define CHASSIS_WHEEL_TRACK_M                          (0.195f) /* 左右轮距，单位：米 */
#define CHASSIS_WHEEL_DIAMETER_M                       (0.065f) /* 车轮直径，单位：米 */
#define RIGHT_CIRCLE_DIAMETER_M                        (1.000f) /* 右转圆周演示的中心轨迹直径，单位：米 */
#define RIGHT_CIRCLE_DEMO_CENTER_RPM                   (120.0f) /* 右转圆周演示的中心轮速，单位：RPM */
#define MOTOR_OUTPUT_SWAP_LEFT_RIGHT                   (0u)     /* 左右电机输出交换：0=不交换，1=交换 */
#define MOTOR_LEFT_OUTPUT_POLARITY                     (-1)     /* 左电机输出极性：1=同向，-1=反向 */
#define MOTOR_RIGHT_OUTPUT_POLARITY                    (-1)     /* 右电机输出极性：1=同向，-1=反向 */
#define WHEEL_LEFT_ENCODER_SIGN                        (1.0f)   /* 左轮前进时编码器符号 */
#define WHEEL_RIGHT_ENCODER_SIGN                       (-1.0f)  /* 右轮前进时编码器符号 */

/* 轮速闭环 */
#define WHEEL_LEFT_KP                                  (80.0f)  /* 左轮比例增益 */
#define WHEEL_LEFT_KI                                  (8.0f)   /* 左轮积分增益 */
#define WHEEL_LEFT_KD                                  (0.0f)   /* 左轮微分增益 */
#define WHEEL_LEFT_KS                                  (0.0f)   /* 左轮静摩擦前馈 */
#define WHEEL_LEFT_KV                                  (30.0f)  /* 左轮速度前馈 */
#define WHEEL_LEFT_KA                                  (0.0f)   /* 左轮加速度前馈 */
#define WHEEL_RIGHT_KP                                 (80.0f)  /* 右轮比例增益 */
#define WHEEL_RIGHT_KI                                 (8.0f)   /* 右轮积分增益 */
#define WHEEL_RIGHT_KD                                 (0.0f)   /* 右轮微分增益 */
#define WHEEL_RIGHT_KS                                 (0.0f)   /* 右轮静摩擦前馈 */
#define WHEEL_RIGHT_KV                                 (33.0f)  /* 右轮速度前馈 */
#define WHEEL_RIGHT_KA                                 (0.0f)   /* 右轮加速度前馈 */
#define WHEEL_PID_INTEGRAL_LIMIT                       (6000.0f)/* 轮速积分项绝对值上限 */
#define WHEEL_PID_OUTPUT_LIMIT                         (10000.0f)/* 轮速控制输出绝对值上限 */
#define WHEEL_TARGET_RPM_LIMIT                         (250.0f) /* 轮速目标绝对值上限，单位：RPM */
#define WHEEL_FEEDFORWARD_ACCEL_RPM_S_LIMIT            (600.0f) /* 前馈目标加速度上限，单位：RPM 每秒 */
#define WHEEL_LEFT_PWM_MAP_SCALE                       (1.00f)  /* 左电机控制量到 PWM 的映射比例 */
#define WHEEL_RIGHT_PWM_MAP_SCALE                      (0.92f)  /* 右电机控制量到 PWM 的映射比例 */
#define WHEEL_PWM_SLEW_DUTY_PER_S                      (30000.0f)/* PWM 占空比变化率上限，单位：每秒 */
#define WHEEL_RAPID_BRAKE_PWM_SLEW_DUTY_PER_S         (150000.0f)/* 模式1减速时PWM变化率上限 */
#define WHEEL_ACCEL_FILTER_ALPHA                       (0.20f)  /* 编码器加速度低通滤波系数 */
#define WHEEL_MEASURED_ACCEL_MPS2_LIMIT                (20.0f)  /* 编码器测得加速度绝对值上限，单位：米每平方秒 */

/* 黑线查表循迹 */
#define LINE_BLACK_ACTIVE_LEVEL                        (1u)     /* 黑线传感器有效电平 */
#define LINE_SENSOR_WIDE_PATTERN_MIN_COUNT             (4u)     /* 字母/横线宽图案过滤所需最少有效传感器数 */
#define LINE_SENSOR_MARKER_MIN_COUNT                   (6u)     /* 赛道标记判定所需最少有效传感器数 */
#define LINE_LOOKUP_CORRECTION_SIGN                    (1.0f)   /* 转向修正方向 */
#define LINE_LOOKUP_INITIAL_PHASE                      (0u)     /* 初始赛段：0=直线，1=右弧 */
#define LINE_LOOKUP_STRAIGHT_LENGTH_M                  (1.500f) /* 单段直线长度，单位：米 */
#define LINE_LOOKUP_ARC_RADIUS_M                       (0.500f) /* 弧线半径，单位：米 */
#define LINE_LOOKUP_ARC_LENGTH_M                       (1.57079633f) /* 单段弧线长度，单位：米 */
#define LINE_LOOKUP_TRANSITION_HALF_LENGTH_M           (0.150f) /* 赛段切换过渡半长，单位：米 */
#define LINE_LOOKUP_SPEED_REFERENCE_RPM                (120.0f) /* 查表反馈标定基准轮速，单位：RPM */
#define LINE_LOOKUP_STRAIGHT_BASE_RPM                  (170.0f) /* 直线基础轮速，单位：RPM */
#define LINE_LOOKUP_SPEED_MIN_RPM                      (60.0f)  /* 运行轮速下限，单位：RPM */
#define LINE_LOOKUP_SPEED_MAX_RPM                      (180.0f) /* 运行轮速上限，单位：RPM */
#define LINE_LOOKUP_FEEDBACK_SCALE_MAX                 (1.00f)  /* 反馈修正最大缩放比例 */
#define LINE_LOOKUP_LEVEL_1_TURN_RPM                   (8.0f)   /* 一级偏差转向差速，单位：RPM */
#define LINE_LOOKUP_LEVEL_2_TURN_RPM                   (16.0f)  /* 二级偏差转向差速，单位：RPM */
#define LINE_LOOKUP_LEVEL_3_TURN_RPM                   (28.0f)  /* 三级偏差转向差速，单位：RPM */
#define LINE_LOOKUP_LEVEL_4_TURN_RPM                   (42.0f)  /* 四级偏差转向差速，单位：RPM */
#define LINE_LOOKUP_TURN_RPM_LIMIT                     (55.0f)  /* 转向差速绝对值上限，单位：RPM */
#define LINE_LOOKUP_BASE_START_SLEW_RPM_PER_S          (180.0f) /* 基础轮速加速度上限，单位：RPM 每秒 */
#define LINE_LOOKUP_BASE_JERK_RPM_PER_S2               (720.0f) /* S 型启停加加速度上限，单位：RPM 每平方秒 */
#define LINE_LOOKUP_TURN_SLEW_RPM_PER_S                (240.0f) /* 转向差速变化率，单位：RPM 每秒 */
#define LINE_LOOKUP_REVERSE_HOLD_MS                    (40u)    /* 转向反向前保持时间，单位：毫秒 */
#define LINE_LOOKUP_LOST_HOLD_MS                       (80u)    /* 丢线后保持原指令时间，单位：毫秒 */
#define LINE_LOOKUP_SEARCH_TIMEOUT_MS                  (250u)   /* 丢线搜索超时时间，单位：毫秒 */
#define LINE_LOOKUP_SEARCH_TURN_RPM                    (35.0f)  /* 丢线搜索转向差速，单位：RPM */

/* 配置合法性检查 */
#if ((MOTOR_APP_AUTO_START_LINE_FOLLOW != 0u) && \
     (MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO != 0u))
#error "Only one motor app auto-start mode may be enabled"
#endif
#if ((UART3_MAIX_MODE != UART3_MAIX_MODE_NORMAL) && \
     (UART3_MAIX_MODE != UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG) && \
     (UART3_MAIX_MODE != UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG))
#error "UART3_MAIX_MODE is invalid"
#endif
#if ((EMM42_BALANCE_DEMO_ENABLE != 0u) && (EMM42_BALANCE_DEMO_ENABLE != 1u))
#error "EMM42_BALANCE_DEMO_ENABLE must be 0 or 1"
#endif
#if ((BALL_RETURN_DEMO_ENABLE != 0u) && (BALL_RETURN_DEMO_ENABLE != 1u))
#error "BALL_RETURN_DEMO_ENABLE must be 0 or 1"
#endif
#if ((BALANCE_CONTROL_ENABLE != 0u) && (BALANCE_CONTROL_ENABLE != 1u))
#error "BALANCE_CONTROL_ENABLE must be 0 or 1"
#endif
#if ((BALANCE_SIMPLE_CONTROL_ENABLE != 0u) && (BALANCE_SIMPLE_CONTROL_ENABLE != 1u))
#error "BALANCE_SIMPLE_CONTROL_ENABLE must be 0 or 1"
#endif
#if ((BALANCE_DRIVE_DEMO_ENABLE != 0u) && (BALANCE_DRIVE_DEMO_ENABLE != 1u))
#error "BALANCE_DRIVE_DEMO_ENABLE must be 0 or 1"
#endif
#if ((BALANCE_DRIVE_DEMO_ENABLE != 0u) && \
     (BALANCE_CONTROL_ENABLE == 0u) && \
     (BALANCE_SIMPLE_CONTROL_ENABLE == 0u))
#error "Balance drive demo requires a balance controller"
#endif
#if ((BALANCE_STARTUP_CALIBRATED != 0u) && (BALANCE_STARTUP_CALIBRATED != 1u))
#error "BALANCE_STARTUP_CALIBRATED must be 0 or 1"
#endif
#if ((BALANCE_SIMPLE_RUNTIME_MOTOR_SAFETY_ENABLE != 0u) && \
     (BALANCE_SIMPLE_RUNTIME_MOTOR_SAFETY_ENABLE != 1u))
#error "BALANCE_SIMPLE_RUNTIME_MOTOR_SAFETY_ENABLE must be 0 or 1"
#endif
#if ((BALANCE_SIMPLE_STARTUP_ACK_FALLBACK_ENABLE != 0u) && \
     (BALANCE_SIMPLE_STARTUP_ACK_FALLBACK_ENABLE != 1u))
#error "BALANCE_SIMPLE_STARTUP_ACK_FALLBACK_ENABLE must be 0 or 1"
#endif
#if ((BALANCE_CONTROL_ENABLE + BALANCE_SIMPLE_CONTROL_ENABLE + \
      EMM42_BALANCE_DEMO_ENABLE + BALL_RETURN_DEMO_ENABLE) > 1u)
#error "Balance controller and actuator demos are mutually exclusive"
#endif
#if ((UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG) && \
     (BALANCE_CONTROL_ENABLE == 0u) && (EMM42_BALANCE_DEMO_ENABLE == 0u) && \
     (BALL_RETURN_DEMO_ENABLE == 0u) && (BALANCE_SIMPLE_CONTROL_ENABLE == 0u))
#error "Balance telemetry mode requires balance control or an actuator demo"
#endif
#if ((MOTOR_OUTPUT_SWAP_LEFT_RIGHT != 0u) && (MOTOR_OUTPUT_SWAP_LEFT_RIGHT != 1u))
#error "MOTOR_OUTPUT_SWAP_LEFT_RIGHT must be 0 or 1"
#endif
#if ((MOTOR_LEFT_OUTPUT_POLARITY != 1) && (MOTOR_LEFT_OUTPUT_POLARITY != -1))
#error "MOTOR_LEFT_OUTPUT_POLARITY must be 1 or -1"
#endif
#if ((MOTOR_RIGHT_OUTPUT_POLARITY != 1) && (MOTOR_RIGHT_OUTPUT_POLARITY != -1))
#error "MOTOR_RIGHT_OUTPUT_POLARITY must be 1 or -1"
#endif
#if (LINE_LOOKUP_INITIAL_PHASE > 1u)
#error "LINE_LOOKUP_INITIAL_PHASE must be 0 or 1"
#endif
#if (LINE_LOOKUP_LOST_HOLD_MS >= LINE_LOOKUP_SEARCH_TIMEOUT_MS)
#error "Line lost hold time must be shorter than search timeout"
#endif

#endif
