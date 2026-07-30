#ifndef CONTROL_CONFIG_H_
#define CONTROL_CONFIG_H_

/* 底盘控制公共配置，100 Hz 周期；请在实车上标定。 */
#define CHASSIS_CONTROL_PERIOD_MS             (10u)

/* 测试固件上电保持电机停机，不自动进入循迹或圆周 demo。 */
#define MOTOR_APP_AUTO_START_LINE_FOLLOW      (0u)
#define MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO (0u)

#if ((MOTOR_APP_AUTO_START_LINE_FOLLOW != 0u) && \
     (MOTOR_APP_AUTO_START_RIGHT_CIRCLE_DEMO != 0u))
#error "Only one motor app auto-start mode may be enabled"
#endif

/* 实测车体几何参数，以及顺时针 1 m 直径圆的中心轨迹。 */
#define CHASSIS_WHEEL_TRACK_M                  (0.18f)
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
#define WHEEL_LEFT_KV                         (0.0f)
#define WHEEL_LEFT_KA                         (0.0f)

#define WHEEL_RIGHT_KP                        (80.0f)
#define WHEEL_RIGHT_KI                        (8.0f)
#define WHEEL_RIGHT_KD                        (0.0f)
#define WHEEL_RIGHT_KS                        (0.0f)
#define WHEEL_RIGHT_KV                        (0.0f)
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

/*
 * 模块 OUT 信号已是数字量（0/1），MCU 侧无电压阈值。
 * 请在传感器模块上调节比较器阈值。
 * 现有位置 PID 按白色电平 0 的补集误差完成实车调参，不得直接翻转；
 * 物理黑线电平 1 单独用于右弯和横线判断。
 */
#define LINE_SENSOR_ACTIVE_LEVEL              (0u)
#define LINE_BLACK_ACTIVE_LEVEL               (1u)
#define LINE_SENSOR_MARKER_MIN_COUNT          (6u)
#define LINE_LOST_HOLD_MS                     (200u)

/* 传感器误差范围 -3500 ... +3500，正值朝向通道 7。 */
#define LINE_STEERING_SIGN                    (-1.0f)
#define LINE_ERROR_FILTER_ALPHA               (0.30f)
#define LINE_TARGET_SLEW_RPM_PER_S            (500.0f)  /* 目标轮速变化率上限（RPM/s），限制加减速 */
#define LINE_BASE_RPM_DEFAULT                 (170.0f)   /* 循迹直行基准速度（RPM），主速度调节项 */
#define LINE_MIN_RPM_DEFAULT                  (90.0f)   /* 大偏差/急弯时的最低基准速度（RPM） */
#define LINE_TURN_RPM_LIMIT                   (60.0f)   /* 转向 PID 输出的差速上限（RPM） */
/* 模块标号第 6~8 路对应 values[] 的 0 基下标 5~7。 */
#define LINE_RIGHT_SENSOR_FIRST_INDEX         (5u)
#define LINE_RIGHT_SENSOR_LAST_INDEX          (7u)
#define LINE_RIGHT_CURVE_DETECT_MS            (60u)
#if ((LINE_RIGHT_SENSOR_FIRST_INDEX > LINE_RIGHT_SENSOR_LAST_INDEX) || \
     (LINE_RIGHT_SENSOR_LAST_INDEX >= 8u))
#error "Right line sensor range must be within channels 0..7"
#endif
#define LINE_KP                               (0.015f)
#define LINE_KI                               (0.0f)
#define LINE_KD                               (0.00018f)
#define LINE_INTEGRAL_LIMIT                   (3000.0f)

#endif
