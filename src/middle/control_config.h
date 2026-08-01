#ifndef CONTROL_CONFIG_H_
#define CONTROL_CONFIG_H_

/* 底盘控制公共配置，100 Hz 周期；请在实车上标定。 */
#define CHASSIS_CONTROL_PERIOD_MS             (10u)

/* 通信联调阶段上电保持底盘停机；与 RIGHT_CIRCLE_DEMO 二选一。 */
#define MOTOR_APP_AUTO_START_LINE_FOLLOW      (0u)
#define MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO (0u)

/*
 * UART3 工作模式。正常模式只接收 Maix 视觉帧，TX 完全静默；另外两项仅用于
 * 100 Hz 台架遥测调试，不属于正常运行通信。
 */
#define UART3_MAIX_MODE_NORMAL                    (0u)
#define UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG   (1u)
#define UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG   (2u)
#define UART3_MAIX_MODE                            (UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG)

/* V1 static center stability closed-loop; demo is mutually exclusive. */
#ifndef EMM42_BALANCE_DEMO_ENABLE
#define EMM42_BALANCE_DEMO_ENABLE              (0u)
#endif
#ifndef BALANCE_CONTROL_ENABLE
#define BALANCE_CONTROL_ENABLE                 (1u)
#endif

/*
 * Measure the physical negative-angle startup stop before setting this flag.
 * Keeping CALIBRATED at zero makes the firmware initialize UART7 but never
 * enable or move the balance actuator.
 */
#ifndef BALANCE_STARTUP_CALIBRATED
#define BALANCE_STARTUP_CALIBRATED             (1u)
#endif
#ifndef BALANCE_STARTUP_LEVER_ANGLE_DEG
#define BALANCE_STARTUP_LEVER_ANGLE_DEG        (-5.0f)
#endif

#define BALANCE_ESTIMATOR_PERIOD_MS            (5u)
#define BALANCE_OUTER_CONTROL_PERIOD_MS        (20u)
/* Compatibility name for the 200 Hz estimator/application tick. */
#define BALANCE_CONTROL_PERIOD_MS              BALANCE_ESTIMATOR_PERIOD_MS
#define BALANCE_COMMAND_PERIOD_MS              (20u)
#define BALANCE_POSITION_QUERY_PERIOD_MS       (100u)
#define BALANCE_POWER_WAIT_MS                  (3000u)
#define BALANCE_LOWER_STOP_SETTLE_MS            (1500u)
#define BALANCE_COMMAND_TIMEOUT_MS             (25u)
#define BALANCE_MOVE_LEVEL_TIMEOUT_MS          (2500u)
#define BALANCE_LEVEL_SETTLE_MS                (200u)
#define BALANCE_HARD_EDGE_TIMEOUT_MS           (200u)
#define BALANCE_MAX_CONSECUTIVE_COMMAND_ERRORS (3u)
#define BALANCE_RECOVERY_VALID_FRAMES          (5u)
#define BALANCE_MIN_VISION_CONFIDENCE          (50u)

/* Cascaded ball controller: position error -> velocity -> acceleration. */
#define BALANCE_POSITION_LOOP_GAIN_S_INV       (2.0f)
#define BALANCE_VELOCITY_LOOP_GAIN_S_INV       (2.0f)
#define BALANCE_MAX_BALL_VELOCITY_MPS          (0.060f)
#define BALANCE_LOW_SPEED_FRICTION_ACCEL_MPS2  (0.045f)
#define BALANCE_CENTER_CAPTURE_POSITION_M      (0.004f)
#define BALANCE_LOW_SPEED_THRESHOLD_MPS        (0.010f)
#define BALANCE_ESTIMATOR_POSITION_GAIN        (0.65f)
#define BALANCE_ESTIMATOR_VELOCITY_GAIN        (0.50f)
#define BALANCE_MAX_BALL_ACCEL_MPS2            (0.45f)
#define BALANCE_MAX_LEVER_ANGLE_DEG            (4.0f)
#define BALANCE_DEGRADED_LEVER_ANGLE_DEG       (2.0f)
#define BALANCE_MAX_LEVER_RATE_DEG_S           (30.0f)
#define BALANCE_MAX_LEVER_ACCEL_DEG_S2         (600.0f)
#define BALANCE_LEVER_COMMAND_DEADBAND_DEG      (0.1f)
#define BALANCE_EDGE_POSITION_M                (0.100f)
#define BALANCE_HARD_EDGE_POSITION_M           (0.125f)
#define BALANCE_EDGE_RECOVERY_ACCEL_MPS2       (0.22f)
#define BALANCE_FRESH_MEASUREMENT_MS           (30u)
#define BALANCE_VALID_MEASUREMENT_MS           (80u)
#define BALANCE_VISION_TRANSPORT_LATENCY_MS     (3u)
#define BALANCE_VISION_MAX_COMPENSATION_MS      (30u)
#define BALANCE_CAR_ACCEL_FEEDFORWARD_GAIN      (1.0f)
#define BALANCE_CAR_ACCEL_FEEDFORWARD_SIGN      (1.0f)
#define BALANCE_CAR_ACCEL_LIMIT_MPS2            (2.0f)

/* Ball reference profile; leave feedback headroom below the 0.45m/s2 limit. */
#define BALANCE_PROFILE_DRIVE_ACCEL_MPS2       (0.12f)
#define BALANCE_PROFILE_BRAKE_ACCEL_MPS2       (0.16f)
#define BALANCE_PROFILE_MAX_VELOCITY_MPS       (0.060f)
/* Lead the reference trajectory to compensate vision and actuator delay. */
#define BALANCE_PROFILE_BRAKE_LOOKAHEAD_S       (0.12f)
#define BALANCE_PROFILE_ACCEL_FF_GAIN           (1.00f)
#define BALANCE_PROFILE_POSITION_TOLERANCE_M   (0.0005f)
#define BALANCE_PROFILE_VELOCITY_TOLERANCE_MPS (0.002f)
#define BALANCE_TARGET_POSITION_LIMIT_M        (0.090f)
#define BALANCE_SEQUENCE_POSITIVE_TARGET_M     (0.050f)
#define BALANCE_SEQUENCE_NEGATIVE_TARGET_M     (-0.050f)
#define BALANCE_SEQUENCE_POSITION_TOLERANCE_M  (0.006f)
#define BALANCE_SEQUENCE_VELOCITY_TOLERANCE_MPS (0.030f)
#define BALANCE_SEQUENCE_SETTLE_MS             (100u)
#define BALANCE_SEQUENCE_TIMEOUT_MS            (4800u)

#define BALANCE_EMM42_MOVE_RPM                 (30u)
#define BALANCE_EMM42_ACCELERATION             (20u)
/* This installation raises the lever when the Emm42 shaft angle is negative. */
#define BALANCE_EMM42_DIRECTION_SIGN           (-1)
/* Logical positive lever angle is opposite to the linkage model alpha axis. */
#define BALANCE_LINKAGE_TARGET_SIGN            (-1)
/* Closed-loop correction direction; calibrated from actual ball response. */
#define BALANCE_CONTROL_OUTPUT_SIGN             (-1)
#define BALANCE_LEVEL_MOTOR_TOLERANCE_DEG      (1.0f)
#define BALANCE_MOTOR_FOLLOW_ERROR_DEG         (5.0f)
#define BALANCE_MOTOR_FOLLOW_ERROR_TIMEOUT_MS  (1000u)

#if ((MOTOR_APP_AUTO_START_LINE_FOLLOW != 0u) && \
     (MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO != 0u))
#error "Only one motor app auto-start mode may be enabled"
#endif
#if ((UART3_MAIX_MODE != UART3_MAIX_MODE_NORMAL) && \
     (UART3_MAIX_MODE != UART3_MAIX_MODE_CHASSIS_TELEMETRY_DEBUG) && \
     (UART3_MAIX_MODE != UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG))
#error "UART3_MAIX_MODE is invalid"
#endif
#if ((EMM42_BALANCE_DEMO_ENABLE != 0u) && \
     (EMM42_BALANCE_DEMO_ENABLE != 1u))
#error "EMM42_BALANCE_DEMO_ENABLE must be 0 or 1"
#endif
#if ((BALANCE_CONTROL_ENABLE != 0u) && (BALANCE_CONTROL_ENABLE != 1u))
#error "BALANCE_CONTROL_ENABLE must be 0 or 1"
#endif
#if ((BALANCE_STARTUP_CALIBRATED != 0u) && \
     (BALANCE_STARTUP_CALIBRATED != 1u))
#error "BALANCE_STARTUP_CALIBRATED must be 0 or 1"
#endif
#if ((BALANCE_EMM42_DIRECTION_SIGN != 1) && \
     (BALANCE_EMM42_DIRECTION_SIGN != -1))
#error "BALANCE_EMM42_DIRECTION_SIGN must be 1 or -1"
#endif
#if ((BALANCE_LINKAGE_TARGET_SIGN != 1) && \
     (BALANCE_LINKAGE_TARGET_SIGN != -1))
#error "BALANCE_LINKAGE_TARGET_SIGN must be 1 or -1"
#endif
#if ((BALANCE_CONTROL_OUTPUT_SIGN != 1) && \
     (BALANCE_CONTROL_OUTPUT_SIGN != -1))
#error "BALANCE_CONTROL_OUTPUT_SIGN must be 1 or -1"
#endif
#if ((BALANCE_CONTROL_ENABLE != 0u) && (EMM42_BALANCE_DEMO_ENABLE != 0u))
#error "Balance controller and EMM42 demo are mutually exclusive"
#endif
#if ((UART3_MAIX_MODE == UART3_MAIX_MODE_BALANCE_TELEMETRY_DEBUG) && \
     (BALANCE_CONTROL_ENABLE == 0u) && (EMM42_BALANCE_DEMO_ENABLE == 0u))
#error "Balance telemetry mode requires balance control or EMM42 demo"
#endif

/* 实测车体几何参数，以及顺时针 1 m 直径圆的中心轨迹。 */
#define CHASSIS_WHEEL_TRACK_M                  (0.195f)
#define CHASSIS_WHEEL_DIAMETER_M               (0.065f)
#define RIGHT_CIRCLE_DIAMETER_M                (1.000f)
#define RIGHT_CIRCLE_DEMO_CENTER_RPM           (120.0f)

/*
 * 逻辑轮速命令与 TB6612 接线的标定。
 * 仅当 PWMA/PWMB 仍驱动相反物理轮时，将 SWAP 设为 1。
 * 若正命令使该轮反向旋转，则将该轮极性设为 -1。
 */
#define MOTOR_OUTPUT_SWAP_LEFT_RIGHT          (0u)
#define MOTOR_LEFT_OUTPUT_POLARITY            (-1)
#define MOTOR_RIGHT_OUTPUT_POLARITY           (-1)

#if ((MOTOR_OUTPUT_SWAP_LEFT_RIGHT != 0u) && \
     (MOTOR_OUTPUT_SWAP_LEFT_RIGHT != 1u))
#error "MOTOR_OUTPUT_SWAP_LEFT_RIGHT must be 0 or 1"
#endif
#if ((MOTOR_LEFT_OUTPUT_POLARITY != 1) && \
     (MOTOR_LEFT_OUTPUT_POLARITY != -1))
#error "MOTOR_LEFT_OUTPUT_POLARITY must be 1 or -1"
#endif
#if ((MOTOR_RIGHT_OUTPUT_POLARITY != 1) && \
     (MOTOR_RIGHT_OUTPUT_POLARITY != -1))
#error "MOTOR_RIGHT_OUTPUT_POLARITY must be 1 or -1"
#endif

/* 编码器符号：前进时车轮转速应上报为正 RPM。 */
#define WHEEL_LEFT_ENCODER_SIGN               (1.0f)
#define WHEEL_RIGHT_ENCODER_SIGN              (-1.0f)

/* 轮速环初始增益：PWM 占空比与 RPM 域量之间的比例。 */
#define WHEEL_LEFT_KP                         (80.0f)
#define WHEEL_LEFT_KI                         (8.0f)
#define WHEEL_LEFT_KD                         (0.0f)
#define WHEEL_LEFT_KS                         (0.0f)
#define WHEEL_LEFT_KV                         (30.0f)
#define WHEEL_LEFT_KA                         (0.0f)

#define WHEEL_RIGHT_KP                        (80.0f)
#define WHEEL_RIGHT_KI                        (8.0f)
#define WHEEL_RIGHT_KD                        (0.0f)
#define WHEEL_RIGHT_KS                        (0.0f)
#define WHEEL_RIGHT_KV                        (33.0f)
#define WHEEL_RIGHT_KA                        (0.0f)

#define WHEEL_PID_INTEGRAL_LIMIT              (6000.0f)
#define WHEEL_PID_OUTPUT_LIMIT                (10000.0f)
/* 电机空载额定最高转速；软件目标应低于此值。 */
#define MOTOR_RATED_MAX_RPM                   (320.0f)
#define WHEEL_LEFT_MEASURED_MAX_RPM           (305.0f)
#define WHEEL_RIGHT_MEASURED_MAX_RPM          (330.0f)
#define WHEEL_TARGET_RPM_LIMIT                (250.0f)
#define WHEEL_FEEDFORWARD_ACCEL_RPM_S_LIMIT   (600.0f)
/* 控制器输出到各电机的映射；右轮 10000 对应 PWM 9200。 */
#define WHEEL_LEFT_PWM_MAP_SCALE              (1.00f)
#define WHEEL_RIGHT_PWM_MAP_SCALE             (0.92f)
#define WHEEL_PWM_SLEW_DUTY_PER_S             (30000.0f)
/* 编码器差分前进加速度低通和异常值限幅。 */
#define WHEEL_ACCEL_FILTER_ALPHA               (0.20f)
#define WHEEL_MEASURED_ACCEL_MPS2_LIMIT        (20.0f)

/* Digital black-line lookup controller for the clockwise capsule track. */
#define LINE_BLACK_ACTIVE_LEVEL               (1u)
#define LINE_SENSOR_MARKER_MIN_COUNT          (6u)
#define LINE_LOOKUP_CORRECTION_SIGN           (1.0f)
#define LINE_LOOKUP_INITIAL_PHASE             (0u)       /* 0=straight, 1=right arc */

#define LINE_LOOKUP_STRAIGHT_LENGTH_M         (1.500f)
#define LINE_LOOKUP_ARC_RADIUS_M              (0.500f)
#define LINE_LOOKUP_ARC_LENGTH_M              (1.57079633f)
#define LINE_LOOKUP_TRANSITION_HALF_LENGTH_M  (0.150f)

/* 120 RPM 是查表反馈标定基准；运行中中心速度保持恒定。 */
#define LINE_LOOKUP_SPEED_REFERENCE_RPM       (120.0f)
#define LINE_LOOKUP_STRAIGHT_BASE_RPM         (170.0f)
#define LINE_LOOKUP_SPEED_MIN_RPM             (60.0f)
#define LINE_LOOKUP_SPEED_MAX_RPM             (180.0f)
#define LINE_LOOKUP_FEEDBACK_SCALE_MAX        (1.00f)
#define LINE_LOOKUP_LEVEL_1_TURN_RPM          (8.0f)
#define LINE_LOOKUP_LEVEL_2_TURN_RPM          (16.0f)
#define LINE_LOOKUP_LEVEL_3_TURN_RPM          (28.0f)
#define LINE_LOOKUP_LEVEL_4_TURN_RPM          (42.0f)
#define LINE_LOOKUP_TURN_RPM_LIMIT            (55.0f)
#define LINE_LOOKUP_BASE_START_SLEW_RPM_PER_S (300.0f)
#define LINE_LOOKUP_TURN_SLEW_RPM_PER_S       (240.0f)
#define LINE_LOOKUP_REVERSE_HOLD_MS           (40u)

#define LINE_LOOKUP_LOST_HOLD_MS              (80u)
#define LINE_LOOKUP_SEARCH_TIMEOUT_MS         (250u)
#define LINE_LOOKUP_SEARCH_TURN_RPM           (35.0f)

#if (LINE_LOOKUP_INITIAL_PHASE > 1u)
#error "LINE_LOOKUP_INITIAL_PHASE must be 0 or 1"
#endif
#if (LINE_LOOKUP_LOST_HOLD_MS >= LINE_LOOKUP_SEARCH_TIMEOUT_MS)
#error "Line lost hold time must be shorter than search timeout"
#endif
#endif
