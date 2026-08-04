#include "wheel_speed_control.h"

#include "control_config.h"
#include "control_pid.h"
#include "encoder.h"
#include "motor.h"

typedef struct
{
    control_pid_t pid;
    float ks;
    float kv;
    float ka;
    float target_rpm;
    float previous_target_rpm;
    float applied_output;
    float encoder_sign;
} wheel_controller_t;

static wheel_controller_t wheel_left;
static wheel_controller_t wheel_right;
static wheel_speed_control_status_t wheel_status;
static float wheel_previous_planned_speed_mps;
static float wheel_previous_measured_speed_mps;
static float wheel_filtered_measured_accel_mps2;
static uint8 wheel_kinematics_initialized;
static uint8 wheel_rapid_brake_enabled;

#define WHEEL_PI_F                         (3.14159265f)

static float wheel_clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static int32 wheel_round_to_int(float value)
{
    return (int32)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static float wheel_slew(float current, float target, float max_delta)
{
    if (target > (current + max_delta))
    {
        return current + max_delta;
    }
    if (target < (current - max_delta))
    {
        return current - max_delta;
    }
    return target;
}

static float wheel_rpm_to_mps(float rpm)
{
    return rpm * WHEEL_PI_F * CHASSIS_WHEEL_DIAMETER_M / 60.0f;
}

static void wheel_reset_outputs(void)
{
    control_pid_reset(&wheel_left.pid);
    control_pid_reset(&wheel_right.pid);
    wheel_left.target_rpm = 0.0f;
    wheel_right.target_rpm = 0.0f;
    wheel_left.previous_target_rpm = 0.0f;
    wheel_right.previous_target_rpm = 0.0f;
    wheel_left.applied_output = 0.0f;
    wheel_right.applied_output = 0.0f;
    wheel_status.left_target_rpm = 0.0f;
    wheel_status.right_target_rpm = 0.0f;
    wheel_status.left_feedforward_pwm = 0.0f;
    wheel_status.right_feedforward_pwm = 0.0f;
    wheel_status.left_feedback_pwm = 0.0f;
    wheel_status.right_feedback_pwm = 0.0f;
    wheel_status.left_duty = 0;
    wheel_status.right_duty = 0;
    wheel_status.left_saturated = 0u;
    wheel_status.right_saturated = 0u;
}

static void wheel_update_kinematics(float dt_s)
{
    float raw_accel;

    wheel_status.planned_speed_mps = wheel_rpm_to_mps(
        0.5f * (wheel_status.left_target_rpm +
                wheel_status.right_target_rpm));
    wheel_status.measured_speed_mps = wheel_rpm_to_mps(
        0.5f * (wheel_status.left_measured_rpm +
                wheel_status.right_measured_rpm));

    if (0u == wheel_kinematics_initialized)
    {
        wheel_status.planned_accel_mps2 = 0.0f;
        wheel_filtered_measured_accel_mps2 = 0.0f;
        wheel_kinematics_initialized = 1u;
    }
    else
    {
        wheel_status.planned_accel_mps2 =
            (wheel_status.planned_speed_mps -
             wheel_previous_planned_speed_mps) / dt_s;
        raw_accel = wheel_clamp(
            (wheel_status.measured_speed_mps -
             wheel_previous_measured_speed_mps) / dt_s,
            WHEEL_MEASURED_ACCEL_MPS2_LIMIT);
        wheel_filtered_measured_accel_mps2 +=
            WHEEL_ACCEL_FILTER_ALPHA *
            (raw_accel - wheel_filtered_measured_accel_mps2);
    }

    wheel_status.measured_accel_mps2 =
        wheel_filtered_measured_accel_mps2;
    wheel_status.kinematics_valid = wheel_kinematics_initialized;
    wheel_previous_planned_speed_mps = wheel_status.planned_speed_mps;
    wheel_previous_measured_speed_mps = wheel_status.measured_speed_mps;
}

static void wheel_init_one(wheel_controller_t *wheel,
                           const control_pid_config_t *pid_config,
                           float ks, float kv, float ka, float encoder_sign)
{
    control_pid_init(&wheel->pid, pid_config);
    wheel->ks = ks;
    wheel->kv = kv;
    wheel->ka = ka;
    wheel->target_rpm = 0.0f;
    wheel->previous_target_rpm = 0.0f;
    wheel->applied_output = 0.0f;
    wheel->encoder_sign = encoder_sign;
}

static float wheel_update_one(wheel_controller_t *wheel, float measured_rpm,
                              float dt_s)
{
    float acceleration;
    float sign_feedforward = 0.0f;
    float feedforward;

    acceleration = wheel_clamp(
        (wheel->target_rpm - wheel->previous_target_rpm) / dt_s,
        WHEEL_FEEDFORWARD_ACCEL_RPM_S_LIMIT);
    if (wheel->target_rpm > 0.0f)
    {
        sign_feedforward = wheel->ks;
    }
    else if (wheel->target_rpm < 0.0f)
    {
        sign_feedforward = -wheel->ks;
    }

    feedforward = sign_feedforward + wheel->kv * wheel->target_rpm +
                  wheel->ka * acceleration;
    wheel->previous_target_rpm = wheel->target_rpm;
    return control_pid_step(&wheel->pid,
        wheel->target_rpm - measured_rpm, feedforward, dt_s);
}

void wheel_speed_control_init(void)
{
    const control_pid_config_t left_config =
    {
        WHEEL_LEFT_KP, WHEEL_LEFT_KI, WHEEL_LEFT_KD,
        WHEEL_PID_INTEGRAL_LIMIT, WHEEL_PID_OUTPUT_LIMIT,
    };
    const control_pid_config_t right_config =
    {
        WHEEL_RIGHT_KP, WHEEL_RIGHT_KI, WHEEL_RIGHT_KD,
        WHEEL_PID_INTEGRAL_LIMIT, WHEEL_PID_OUTPUT_LIMIT,
    };

    wheel_init_one(&wheel_left, &left_config, WHEEL_LEFT_KS,
                   WHEEL_LEFT_KV, WHEEL_LEFT_KA,
                   WHEEL_LEFT_ENCODER_SIGN);
    wheel_init_one(&wheel_right, &right_config, WHEEL_RIGHT_KS,
                   WHEEL_RIGHT_KV, WHEEL_RIGHT_KA,
                   WHEEL_RIGHT_ENCODER_SIGN);
    wheel_speed_control_reset();
}

void wheel_speed_control_reset(void)
{
    wheel_reset_outputs();
    wheel_status.left_measured_rpm = 0.0f;
    wheel_status.right_measured_rpm = 0.0f;
    wheel_status.planned_speed_mps = 0.0f;
    wheel_status.planned_accel_mps2 = 0.0f;
    wheel_status.measured_speed_mps = 0.0f;
    wheel_status.measured_accel_mps2 = 0.0f;
    wheel_status.kinematics_valid = 0u;
    wheel_previous_planned_speed_mps = 0.0f;
    wheel_previous_measured_speed_mps = 0.0f;
    wheel_filtered_measured_accel_mps2 = 0.0f;
    wheel_kinematics_initialized = 0u;
    wheel_rapid_brake_enabled = 0u;
}

void wheel_speed_control_set_target(float left_rpm, float right_rpm)
{
    wheel_left.target_rpm = wheel_clamp(left_rpm, WHEEL_TARGET_RPM_LIMIT);
    wheel_right.target_rpm = wheel_clamp(right_rpm, WHEEL_TARGET_RPM_LIMIT);
}

void wheel_speed_control_set_rapid_brake_enabled(uint8 enabled)
{
    enabled = (0u != enabled) ? 1u : 0u;
    if ((0u != enabled) && (0u == wheel_rapid_brake_enabled))
    {
        control_pid_reset(&wheel_left.pid);
        control_pid_reset(&wheel_right.pid);
    }
    wheel_rapid_brake_enabled = enabled;
}

void wheel_speed_control_update(uint32 period_ms, uint8 enabled)
{
    float dt_s;
    float left_output;
    float right_output;
    float max_output_delta;

    if (0u == period_ms)
    {
        return;
    }

    encoder_update_speed(period_ms);
    wheel_status.left_measured_rpm =
        (float)encoder_get_left_rpm() * wheel_left.encoder_sign;
    wheel_status.right_measured_rpm =
        (float)encoder_get_right_rpm() * wheel_right.encoder_sign;

    dt_s = (float)period_ms * 0.001f;
    if (0u == enabled)
    {
        wheel_reset_outputs();
        wheel_update_kinematics(dt_s);
        motor_stop();
        return;
    }

    left_output = wheel_update_one(&wheel_left,
                                   wheel_status.left_measured_rpm, dt_s);
    right_output = wheel_update_one(&wheel_right,
                                    wheel_status.right_measured_rpm, dt_s);
    left_output = wheel_clamp(left_output * WHEEL_LEFT_PWM_MAP_SCALE,
                              WHEEL_PID_OUTPUT_LIMIT);
    right_output = wheel_clamp(right_output * WHEEL_RIGHT_PWM_MAP_SCALE,
                               WHEEL_PID_OUTPUT_LIMIT);
    max_output_delta = ((0u != wheel_rapid_brake_enabled) ?
        WHEEL_RAPID_BRAKE_PWM_SLEW_DUTY_PER_S :
        WHEEL_PWM_SLEW_DUTY_PER_S) * dt_s;
    wheel_left.applied_output = wheel_slew(wheel_left.applied_output,
        left_output, max_output_delta);
    wheel_right.applied_output = wheel_slew(wheel_right.applied_output,
        right_output, max_output_delta);

    wheel_status.left_target_rpm = wheel_left.target_rpm;
    wheel_status.right_target_rpm = wheel_right.target_rpm;
    wheel_status.left_feedforward_pwm =
        wheel_left.pid.feedforward * WHEEL_LEFT_PWM_MAP_SCALE;
    wheel_status.right_feedforward_pwm =
        wheel_right.pid.feedforward * WHEEL_RIGHT_PWM_MAP_SCALE;
    wheel_status.left_feedback_pwm =
        wheel_left.pid.feedback * WHEEL_LEFT_PWM_MAP_SCALE;
    wheel_status.right_feedback_pwm =
        wheel_right.pid.feedback * WHEEL_RIGHT_PWM_MAP_SCALE;
    wheel_status.left_duty = wheel_round_to_int(wheel_left.applied_output);
    wheel_status.right_duty = wheel_round_to_int(wheel_right.applied_output);
    wheel_status.left_saturated = wheel_left.pid.saturated;
    wheel_status.right_saturated = wheel_right.pid.saturated;
    wheel_update_kinematics(dt_s);
    motor_set_speed(wheel_status.left_duty, wheel_status.right_duty);
}

const wheel_speed_control_status_t *wheel_speed_control_get_status(void)
{
    return &wheel_status;
}
