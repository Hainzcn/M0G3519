#ifndef CONTROL_CONFIG_H_
#define CONTROL_CONFIG_H_

/* 底盘控制公共配置，100 Hz 周期；请在实车上标定。 */
#define CHASSIS_CONTROL_PERIOD_MS             (10u)

/* 通信联调阶段上电保持底盘停机；与 RIGHT_CIRCLE_DEMO 二选一。 */
#define MOTOR_APP_AUTO_START_LINE_FOLLOW      (0u)
#define MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO (0u)

#if ((MOTOR_APP_AUTO_START_LINE_FOLLOW != 0u) && \
     (MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO != 0u))
#error "Only one motor app auto-start mode may be enabled"
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
