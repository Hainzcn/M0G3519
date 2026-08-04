#ifndef CONTROL_CONFIG_H_
#define CONTROL_CONFIG_H_

/* 运行模式 */
#define CHASSIS_CONTROL_PERIOD_MS                    (10u)    /* 底盘控制周期，单位：毫秒 */
#define MOTOR_APP_AUTO_START_LINE_FOLLOW             (0u)     /* 上电自动启动循迹：0=关闭，1=开启 */
#define MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO       (0u)     /* 上电自动启动右转圆周演示：0=关闭，1=开启 */

#define UART3_MAIX_MODE_NORMAL                        (0u)     /* UART3 正常模式：仅接收视觉数据，发送端静默 */
#define UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG       (1u)     /* UART3 底盘遥测调试模式 */
#define UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG       (2u)     /* UART3 摆杆遥测调试模式 */
#define UART3_MAIX_MODE                               (UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG) /* UART3 当前工作模式 */

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
#define BALANCE_DRIVE_DEMO_ENABLE                     (BALANCE_CONTROL_ENABLE) /* 行驶平衡演示：跟随旧版轨迹控制器 */
#endif

/* 摆杆执行器通用上电参数 */
#ifndef BALANCE_STARTUP_CALIBRATED
#define BALANCE_STARTUP_CALIBRATED                    (1u)     /* 机械零位已标定：0=禁止驱动，1=允许驱动 */
#endif
#define BALANCE_POWER_WAIT_MS                          (3000u)  /* 驱动器上电等待时间，单位：毫秒 */
#define BALANCE_LOWER_STOP_SETTLE_MS                   (1500u)  /* 下机械限位稳定时间，单位：毫秒 */
#define BALANCE_LEVEL_SETTLE_MS                        (200u)   /* 回位角稳定时间，单位：毫秒 */
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
#define BALANCE_SIMPLE_POSITION_ON_M                   (0.004f)  /* 启用位置回中控制的误差阈值，单位：米 */
#define BALANCE_SIMPLE_POSITION_OFF_M                  (0.002f)  /* 关闭位置回中控制的误差阈值，单位：米 */
#define BALANCE_SIMPLE_POSITION_KP_S_INV               (1.00f)   /* 位置比例增益，单位：每秒 */
#define BALANCE_SIMPLE_MAX_TARGET_VELOCITY_MPS         (0.060f)  /* 最大目标球速，单位：米每秒 */
#define BALANCE_SIMPLE_BRAKING_ENVELOPE_MPS2           (0.10f)   /* 回中制动包络加速度，单位：米每平方秒 */
#define BALANCE_SIMPLE_ACTUATOR_DELAY_S                (0.25f)   /* 含目标摆角爬升的等效执行延迟，单位：秒 */
#define BALANCE_SIMPLE_VELOCITY_KV_DEG_PER_MM          (0.025f)  /* 球速误差到目标摆角的增益，单位：度/(毫米/秒) */
#define BALANCE_SIMPLE_MAX_TARGET_BEAM_ANGLE_DEG       (2.5f)    /* 目标摆角绝对值上限，单位：度 */
#define BALANCE_SIMPLE_TARGET_BEAM_ANGLE_SLEW_DEG_S    (12.0f)   /* 目标摆角变化率上限，单位：度每秒 */
#define BALANCE_SIMPLE_BEAM_ANGLE_KP_S_INV             (6.0f)    /* 摆角误差到摆杆角速度的比例增益，单位：每秒 */
#define BALANCE_SIMPLE_BEAM_ANGLE_DEADBAND_DEG         (0.08f)   /* 抑制整数 RPM 往返动作的摆角死区 */
#define BALANCE_SIMPLE_MAX_BEAM_VELOCITY_DEG_S         (10.0f)   /* 最大摆杆角速度，单位：度每秒 */
#define BALANCE_SIMPLE_NEAR_POSITION_M                 (0.020f)  /* 近中心增益区间，单位：米 */
#define BALANCE_SIMPLE_NEAR_GAIN                       (0.0f)    /* 近中心附加增益，0 表示关闭 */
#define BALANCE_SIMPLE_NEAR_SCALE_MAX                  (1.50f)   /* 近中心增益最大倍率 */
#define BALANCE_SIMPLE_ANGLE_TRIM_KI_DEG_PER_M_S       (0.0f)    /* 积分摆角偏置关闭 */
#define BALANCE_SIMPLE_ANGLE_TRIM_ZONE_M               (0.06f)  /* 覆盖已观测停驻区的偏置学习范围，单位：米 */
#define BALANCE_SIMPLE_ANGLE_TRIM_LIMIT_DEG            (0.0f)    /* 积分摆角偏置权限关闭，单位：度 */
#define BALANCE_SIMPLE_FIXED_BEAM_BIAS_DEG             (0.50f)   /* 球状态到目标摆角的固定正向偏置，单位：度 */
#define BALANCE_SIMPLE_ACCELERATION_KA                 (0.0f)    /* 球加速度补偿增益，0 表示关闭 */
#define BALANCE_SIMPLE_ACCELERATION_FILTER_ALPHA       (0.20f)   /* 球加速度低通滤波系数 */

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
#define WHEEL_ACCEL_FILTER_ALPHA                       (0.20f)  /* 编码器加速度低通滤波系数 */
#define WHEEL_MEASURED_ACCEL_MPS2_LIMIT                (20.0f)  /* 编码器测得加速度绝对值上限，单位：米每平方秒 */

/* 黑线查表循迹 */
#define LINE_BLACK_ACTIVE_LEVEL                        (1u)     /* 黑线传感器有效电平 */
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
#define LINE_LOOKUP_BASE_START_SLEW_RPM_PER_S          (300.0f) /* 基础轮速启动变化率，单位：RPM 每秒 */
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
#if ((BALANCE_DRIVE_DEMO_ENABLE != 0u) && (BALANCE_CONTROL_ENABLE == 0u))
#error "Balance drive demo requires balance control"
#endif
#if ((BALANCE_STARTUP_CALIBRATED != 0u) && (BALANCE_STARTUP_CALIBRATED != 1u))
#error "BALANCE_STARTUP_CALIBRATED must be 0 or 1"
#endif
#if ((BALANCE_SIMPLE_RUNTIME_MOTOR_SAFETY_ENABLE != 0u) && \
     (BALANCE_SIMPLE_RUNTIME_MOTOR_SAFETY_ENABLE != 1u))
#error "BALANCE_SIMPLE_RUNTIME_MOTOR_SAFETY_ENABLE must be 0 or 1"
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
