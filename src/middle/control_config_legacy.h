#ifndef CONTROL_CONFIG_LEGACY_H_
#define CONTROL_CONFIG_LEGACY_H_

#include "control_config.h"

/* 本文件仅供已停用的旧版轨迹控制器和演示程序兼容编译。 */

#ifndef BALL_RETURN_DEMO_SPEED_SCALE
#define BALL_RETURN_DEMO_SPEED_SCALE                   (1.3f)   /* 开环回球时间轴倍率，大于 1 时更早制动 */
#endif

/* 旧版任务调度与通信 */
#define BALANCE_ESTIMATOR_PERIOD_MS                    (5u)     /* 状态估计周期，单位：毫秒 */
#define BALANCE_OUTER_CONTROL_PERIOD_MS                (20u)    /* 外环控制周期，单位：毫秒 */
#define BALANCE_CONTROL_PERIOD_MS                      BALANCE_ESTIMATOR_PERIOD_MS /* 旧测试兼容名称 */
#define BALANCE_COMMAND_PERIOD_MS                      (20u)    /* 执行器命令周期，单位：毫秒 */
#define BALANCE_POSITION_QUERY_PERIOD_MS               (100u)   /* 电机位置查询周期，单位：毫秒 */
#define BALANCE_COMMAND_TIMEOUT_MS                     (50u)    /* 执行器命令应答超时，单位：毫秒 */
#define BALANCE_MOVE_LEVEL_TIMEOUT_MS                  (2500u)  /* 回位动作超时，单位：毫秒 */
#define BALANCE_HARD_EDGE_TIMEOUT_MS                   (200u)   /* 小球硬边界持续超时，单位：毫秒 */
#define BALANCE_MAX_CONSECUTIVE_COMMAND_ERRORS         (3u)     /* 连续执行器命令错误上限 */
#define BALANCE_MAX_CONSECUTIVE_POSITION_QUERY_ERRORS  (10u)    /* 连续位置查询错误上限 */
#define BALANCE_RECOVERY_VALID_FRAMES                  (5u)     /* 观测恢复所需连续有效帧数 */
#define BALANCE_MIN_VISION_CONFIDENCE                  (50u)    /* 视觉测量最低置信度 */

/* 旧版轨迹控制器 */
#define BALANCE_CALIBRATION_PROVISIONAL                (1u)         /* 参数仍为临时标定值 */
#define BALANCE_POSITION_LOOP_GAIN_S_INV               (1.0f)       /* 位置环增益，单位：每秒 */
#define BALANCE_VELOCITY_LOOP_GAIN_S_INV               (5.0f)       /* 速度环增益，单位：每秒 */
#define BALANCE_MAX_BALL_VELOCITY_MPS                  (0.020f)     /* 最大目标球速，单位：米每秒 */
#define BALANCE_ROLLING_FACTOR                         (0.704013961f) /* 滚动动力学系数 */
#define BALANCE_ROLLING_FRICTION_ACCEL_MPS2            (0.081404074f)/* 等效滚动摩擦加速度，单位：米每平方秒 */
#define BALANCE_RAIL_CURVATURE_M_INV                   (0.201072373f)/* 轨道曲率，单位：每米 */
#define BALANCE_BRAKE_ACCEL_MPS2                       (0.20f)      /* 可用制动加速度，单位：米每平方秒 */
#define BALANCE_ACTUATOR_DELAY_MS                      (20u)        /* 执行器残余延迟，单位：毫秒 */
#define BALANCE_BRAKE_MARGIN_DELAY_MS                  (160u)       /* 制动安全余量延迟，单位：毫秒 */
#define BALANCE_OVERSPEED_RELEASE_RATIO                (0.70f)      /* 超速制动解除比例 */
#define BALANCE_OVERSPEED_MIN_HOLD_MS                  (40u)        /* 超速制动最短保持时间，单位：毫秒 */
#define BALANCE_CENTER_CAPTURE_POSITION_M              (0.004f)     /* 中心捕获位置范围，单位：米 */
#define BALANCE_CENTER_DEAD_POSITION_M                 (0.0015f)    /* 中心位置死区，单位：米 */
#define BALANCE_CAPTURE_VELOCITY_MPS                   (0.008f)     /* 中心捕获速度阈值，单位：米每秒 */
#define BALANCE_STICK_VELOCITY_MPS                     (0.003f)     /* 静摩擦速度阈值，单位：米每秒 */
#define BALANCE_CAPTURE_INTEGRAL_GAIN                  (0.0f)       /* 中心捕获积分增益 */
#define BALANCE_CAPTURE_MAX_ACCEL_MPS2                 (0.05f)      /* 中心捕获最大加速度，单位：米每平方秒 */
#define BALANCE_BREAKAWAY_ANGLE_DEG                    (1.0f)       /* 破静摩擦摆杆角度，单位：度 */
#define BALANCE_BREAKAWAY_QUALIFY_MS                   (100u)       /* 破静摩擦触发确认时间，单位：毫秒 */
#define BALANCE_BREAKAWAY_PULSE_MS                     (40u)        /* 破静摩擦脉冲时间，单位：毫秒 */
#define BALANCE_BREAKAWAY_MOVEMENT_M                   (0.0006f)    /* 破静摩擦成功位移阈值，单位：米 */
#define BALANCE_ESTIMATOR_POSITION_GAIN                (0.65f)      /* 估计器位置残差增益 */
#define BALANCE_ESTIMATOR_VELOCITY_RESIDUAL_GAIN       (0.20f)      /* 估计器速度残差增益 */
#define BALANCE_MAX_BALL_ACCEL_MPS2                    (0.30f)      /* 最大估计球加速度，单位：米每平方秒 */
#define BALANCE_MAX_LEVER_ANGLE_DEG                    (3.0f)       /* 最大摆杆角度，单位：度 */
#define BALANCE_DEGRADED_LEVER_ANGLE_DEG               (2.0f)       /* 降级状态最大摆杆角度，单位：度 */
#define BALANCE_MAX_LEVER_RATE_DEG_S                   (30.0f)      /* 最大摆杆角速度，单位：度每秒 */
#define BALANCE_MAX_LEVER_ACCEL_DEG_S2                 (600.0f)     /* 最大摆杆角加速度，单位：度每平方秒 */
#define BALANCE_LEVER_COMMAND_DEADBAND_DEG             (0.1f)       /* 摆杆命令死区，单位：度 */
#define BALANCE_EDGE_POSITION_M                        (0.100f)     /* 小球边界位置，单位：米 */
#define BALANCE_HARD_EDGE_POSITION_M                   (0.125f)     /* 小球硬边界位置，单位：米 */
#define BALANCE_EDGE_RECOVERY_ACCEL_MPS2               (0.22f)      /* 边界恢复加速度，单位：米每平方秒 */
#define BALANCE_FRESH_MEASUREMENT_MS                   (60u)        /* 新鲜测量最大年龄，单位：毫秒 */
#define BALANCE_VALID_MEASUREMENT_MS                   (100u)       /* 有效测量最大年龄，单位：毫秒 */
#define BALANCE_RECOVERY_MAX_POSITION_STEP_M           (0.010f)     /* 恢复阶段最大位置跳变，单位：米 */
#define BALANCE_VISION_TRANSPORT_LATENCY_MS            (3u)         /* 视觉链路固定延迟，单位：毫秒 */
#define BALANCE_VISION_MAX_COMPENSATION_MS             (30u)        /* 视觉延迟最大补偿时间，单位：毫秒 */

/* 旧版车体加速度前馈与行驶演示 */
#define BALANCE_CAR_IMU_ACCEL_OFFSET_MPS2              (0.200f) /* 车体 IMU X 轴静态零偏，单位：米每平方秒 */
#define BALANCE_CAR_IMU_ACCEL_GAIN                     (1.0f)   /* 车体加速度前馈标定增益 */
#define BALANCE_CAR_IMU_ACCEL_SIGN                     (1.0f)   /* 车体加速度前馈方向 */
#define BALANCE_CAR_IMU_ACCEL_LIMIT_MPS2               (2.0f)   /* 车体加速度前馈绝对值上限，单位：米每平方秒 */
#define BALANCE_CAR_IMU_MAX_AGE_MS                     (25u)    /* IMU 数据最大有效年龄，单位：毫秒 */
#define BALANCE_CAR_FEEDFORWARD_DEBUG_PERIOD_MS        (50u)    /* 前馈调试输出周期，单位：毫秒 */
#define BALANCE_DRIVE_DEMO_LAP_ARM_DISTANCE_M          (4.5f)   /* 圈计数允许触发的最短行驶距离，单位：米 */
#define BALANCE_DRIVE_DEMO_MARKER_DEBOUNCE_MS          (20u)    /* 赛道标记消抖时间，单位：毫秒 */
#define BALANCE_DRIVE_DEMO_TIMEOUT_MS                  (30000u) /* 行驶演示总超时，单位：毫秒 */
#define BALANCE_DRIVE_DEMO_LINE_LOSS_TIMEOUT_MS        (500u)   /* 丢线故障超时，单位：毫秒 */
#define BALANCE_DRIVE_DEMO_IMU_LOSS_TIMEOUT_MS         (100u)   /* IMU 丢失故障超时，单位：毫秒 */

/* 旧版加加速度受限运动轨迹 */
#define BALANCE_PROFILE_DRIVE_ACCEL_MPS2               (0.08f)  /* 加速段加速度，单位：米每平方秒 */
#define BALANCE_PROFILE_BRAKE_ACCEL_MPS2               (0.12f)  /* 制动段加速度，单位：米每平方秒 */
#define BALANCE_PROFILE_MAX_VELOCITY_MPS               (0.020f) /* 轨迹最大速度，单位：米每秒 */
#define BALANCE_PROFILE_MAX_JERK_MPS3                  (2.5f)   /* 最大加加速度，单位：米每立方秒 */
#define BALANCE_PROFILE_FEEDFORWARD_LEAD_S             (0.020f) /* 前馈超前时间，单位：秒 */
#define BALANCE_PROFILE_CAPTURE_POSITION_M             (0.004f) /* 轨迹捕获位置范围，单位：米 */
#define BALANCE_PROFILE_CAPTURE_VELOCITY_MPS           (0.008f) /* 轨迹捕获速度阈值，单位：米每秒 */
#define BALANCE_PROFILE_POSITION_TOLERANCE_M           (0.0005f)/* 轨迹完成位置误差，单位：米 */
#define BALANCE_PROFILE_VELOCITY_TOLERANCE_MPS         (0.002f) /* 轨迹完成速度误差，单位：米每秒 */
#define BALANCE_TARGET_POSITION_LIMIT_M                (0.090f) /* 目标位置绝对值上限，单位：米 */

/* 旧版闭环标定序列 */
#define BALANCE_SEQUENCE_POSITIVE_TARGET_M             (0.030f) /* 正向标定目标，单位：米 */
#define BALANCE_SEQUENCE_NEGATIVE_TARGET_M             (-0.030f)/* 负向标定目标，单位：米 */
#define BALANCE_SEQUENCE_POSITION_TOLERANCE_M          (0.006f) /* 标定序列位置容差，单位：米 */
#define BALANCE_SEQUENCE_VELOCITY_TOLERANCE_MPS        (0.030f) /* 标定序列速度容差，单位：米每秒 */
#define BALANCE_SEQUENCE_SETTLE_MS                     (100u)   /* 标定序列稳定时间，单位：毫秒 */
#define BALANCE_SEQUENCE_TIMEOUT_MS                    (4800u)  /* 单段标定序列超时，单位：毫秒 */
#define BALANCE_LOGICAL_TO_PHYSICAL_LEVER_SIGN         (1)      /* 逻辑摆角到物理摆角的方向 */
#define BALANCE_MOTOR_FOLLOW_ERROR_DEG                 (5.0f)   /* 电机跟随误差阈值，单位：度 */
#define BALANCE_MOTOR_FOLLOW_ERROR_TIMEOUT_MS          (1000u)  /* 电机跟随误差故障时间，单位：毫秒 */

#if ((BALANCE_LOGICAL_TO_PHYSICAL_LEVER_SIGN != 1) && \
     (BALANCE_LOGICAL_TO_PHYSICAL_LEVER_SIGN != -1))
#error "BALANCE_LOGICAL_TO_PHYSICAL_LEVER_SIGN must be 1 or -1"
#endif

#endif
