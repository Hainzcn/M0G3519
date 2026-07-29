#ifndef CONTROL_CONFIG_H_
#define CONTROL_CONFIG_H_

/* Shared 100 Hz chassis-control configuration. Tune on the real vehicle. */
#define CHASSIS_CONTROL_PERIOD_MS             (10u)

/* No start button is fitted yet: enter line-follow mode after power-up. */
#define MOTOR_APP_AUTO_START_LINE_FOLLOW      (1u)

/* Encoder signs must make forward wheel rotation report positive RPM. */
#define WHEEL_LEFT_ENCODER_SIGN               (1.0f)
#define WHEEL_RIGHT_ENCODER_SIGN              (1.0f)

/* Initial wheel-loop gains: PWM duty per RPM-domain quantity. */
#define WHEEL_LEFT_KP                         (100.0f)
#define WHEEL_LEFT_KI                         (8.0f)
#define WHEEL_LEFT_KD                         (0.0f)
#define WHEEL_LEFT_KS                         (0.0f)
#define WHEEL_LEFT_KV                         (0.0f)
#define WHEEL_LEFT_KA                         (0.0f)

#define WHEEL_RIGHT_KP                        (100.0f)
#define WHEEL_RIGHT_KI                        (8.0f)
#define WHEEL_RIGHT_KD                        (0.0f)
#define WHEEL_RIGHT_KS                        (0.0f)
#define WHEEL_RIGHT_KV                        (0.0f)
#define WHEEL_RIGHT_KA                        (0.0f)

#define WHEEL_PID_INTEGRAL_LIMIT              (6000.0f)
#define WHEEL_PID_OUTPUT_LIMIT                (10000.0f)
/* Motor rated no-load maximum; keep the software target below this value. */
#define MOTOR_RATED_MAX_RPM                   (320.0f)
#define WHEEL_TARGET_RPM_LIMIT                (250.0f)
#define WHEEL_FEEDFORWARD_ACCEL_RPM_S_LIMIT   (600.0f)
#define WHEEL_PWM_SLEW_DUTY_PER_S             (30000.0f)

/*
 * The module OUT signal is already digital (0/1), so there is no MCU-side
 * voltage threshold. Adjust the comparator threshold on the sensor module.
 * Bench test shows low reflectance/covering reports logic low, so black line
 * detection uses 0 as the active level.
 */
#define LINE_SENSOR_ACTIVE_LEVEL              (0u)
#define LINE_SENSOR_MARKER_MIN_COUNT          (6u)
#define LINE_LOST_HOLD_MS                     (100u)

/* Sensor error is -3500 ... +3500, positive toward channel 7. */
#define LINE_STEERING_SIGN                    (1.0f)
#define LINE_ERROR_FILTER_ALPHA               (0.20f)
#define LINE_TARGET_SLEW_RPM_PER_S            (300.0f)
#define LINE_BASE_RPM_DEFAULT                 (60.0f)
#define LINE_MIN_RPM_DEFAULT                  (25.0f)
#define LINE_TURN_RPM_LIMIT                   (90.0f)
#define LINE_KP                               (0.025f)
#define LINE_KI                               (0.0f)
#define LINE_KD                               (0.00015f)
#define LINE_INTEGRAL_LIMIT                   (3000.0f)

#endif
