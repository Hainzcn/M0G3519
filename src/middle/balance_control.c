#include "balance_control.h"

#include <math.h>

#define BALANCE_CONTROL_GRAVITY_MPS2        (9.80665f)
#define BALANCE_CONTROL_ROLLING_FACTOR      (5.0f / 7.0f)
#define BALANCE_CONTROL_PI                  (3.14159265358979323846f)
#define BALANCE_CONTROL_DEG_TO_RAD          (BALANCE_CONTROL_PI / 180.0f)
#define BALANCE_CONTROL_RAD_TO_DEG          (180.0f / BALANCE_CONTROL_PI)
#define BALANCE_CONTROL_POSITION_LIMIT_M    (0.13f)
#define BALANCE_CONTROL_VELOCITY_LIMIT_MPS  (5.0f)

static float balance_control_clamp(float value, float low, float high)
{
    if (value > high)
    {
        return high;
    }
    if (value < low)
    {
        return low;
    }
    return value;
}

static float balance_control_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

void balance_control_init(balance_control_t *control,
                          const balance_control_config_t *config)
{
    if ((NULL == control) || (NULL == config))
    {
        return;
    }
    control->config = *config;
    balance_control_reset(control);
}

void balance_control_reset(balance_control_t *control)
{
    if (NULL == control)
    {
        return;
    }
    control->output.has_state = 0u;
    control->output.flags = BALANCE_CONTROL_FLAG_PREDICT_ONLY;
    control->output.estimated_position_m = 0.0f;
    control->output.estimated_velocity_mps = 0.0f;
    control->output.position_error_m = 0.0f;
    control->output.desired_ball_accel_mps2 = 0.0f;
    control->output.lever_angle_deg = 0.0f;
}

void balance_control_step(balance_control_t *control,
                          const balance_control_input_t *input)
{
    balance_control_output_t *output;
    float model_accel;
    float residual;
    float desired_accel = 0.0f;
    float dynamics_limit;
    float radius;
    float lever_rad;
    float lever_deg = 0.0f;
    float angle_limit;
    uint8 flags = 0u;

    if ((NULL == control) || (NULL == input) || (input->dt_s <= 0.0f))
    {
        return;
    }
    output = &control->output;

    if (0u != output->has_state)
    {
        lever_rad = ((0u != input->actual_lever_valid) ?
            input->actual_lever_angle_deg : output->lever_angle_deg) *
            BALANCE_CONTROL_DEG_TO_RAD;
        model_accel = -BALANCE_CONTROL_ROLLING_FACTOR *
            (BALANCE_CONTROL_GRAVITY_MPS2 * sinf(lever_rad) +
             input->car_accel_mps2 * cosf(lever_rad));
        output->estimated_position_m +=
            output->estimated_velocity_mps * input->dt_s +
            0.5f * model_accel * input->dt_s * input->dt_s;
        output->estimated_velocity_mps += model_accel * input->dt_s;
        output->estimated_position_m = balance_control_clamp(
            output->estimated_position_m,
            -BALANCE_CONTROL_POSITION_LIMIT_M,
            BALANCE_CONTROL_POSITION_LIMIT_M);
        output->estimated_velocity_mps = balance_control_clamp(
            output->estimated_velocity_mps,
            -BALANCE_CONTROL_VELOCITY_LIMIT_MPS,
            BALANCE_CONTROL_VELOCITY_LIMIT_MPS);
    }

    if ((0u != input->new_measurement) &&
        (0u != input->measurement_valid))
    {
        if (0u == output->has_state)
        {
            output->estimated_position_m = input->measured_position_m;
            output->estimated_velocity_mps = input->measured_velocity_mps;
            output->has_state = 1u;
        }
        else
        {
            residual = input->measured_position_m -
                output->estimated_position_m;
            output->estimated_position_m +=
                control->config.position_correction_gain * residual;
            output->estimated_velocity_mps +=
                control->config.velocity_correction_gain *
                (input->measured_velocity_mps -
                 output->estimated_velocity_mps);
        }
    }

    if ((0u != input->measurement_valid) &&
        (input->measurement_age_ms <= control->config.fresh_measurement_ms))
    {
        flags |= BALANCE_CONTROL_FLAG_MEASUREMENT_FRESH;
    }
    else
    {
        flags |= BALANCE_CONTROL_FLAG_PREDICT_ONLY;
    }

    output->position_error_m = input->reference_position_m -
        output->estimated_position_m;
    if (0u == input->update_control_output)
    {
        output->flags = (uint8)((output->flags &
            (uint8)(~(BALANCE_CONTROL_FLAG_MEASUREMENT_FRESH |
                       BALANCE_CONTROL_FLAG_PREDICT_ONLY))) | flags);
        return;
    }

    if ((0u != output->has_state) &&
        (input->measurement_age_ms <= control->config.valid_measurement_ms))
    {
        desired_accel = control->config.reference_accel_gain *
            input->reference_accel_mps2 +
            control->config.kp * output->position_error_m +
            control->config.kd * (input->reference_velocity_mps -
                                  output->estimated_velocity_mps);
        if (balance_control_abs(output->estimated_position_m) >=
            control->config.edge_position_m)
        {
            flags |= BALANCE_CONTROL_FLAG_EDGE_RECOVERY;
            desired_accel = (output->estimated_position_m > 0.0f) ?
                -control->config.edge_recovery_accel_mps2 :
                control->config.edge_recovery_accel_mps2;
        }
        if (balance_control_abs(output->estimated_position_m) >=
            control->config.hard_edge_position_m)
        {
            flags |= BALANCE_CONTROL_FLAG_HARD_EDGE;
        }
    }

    if (balance_control_abs(desired_accel) >
        control->config.max_ball_accel_mps2)
    {
        desired_accel = balance_control_clamp(
            desired_accel,
            -control->config.max_ball_accel_mps2,
            control->config.max_ball_accel_mps2);
        flags |= BALANCE_CONTROL_FLAG_DYNAMICS_SATURATED;
    }

    radius = sqrtf(BALANCE_CONTROL_GRAVITY_MPS2 *
                   BALANCE_CONTROL_GRAVITY_MPS2 +
                   input->car_accel_mps2 * input->car_accel_mps2);
    dynamics_limit = -desired_accel /
        (BALANCE_CONTROL_ROLLING_FACTOR * radius);
    if (dynamics_limit > 1.0f)
    {
        dynamics_limit = 1.0f;
        flags |= BALANCE_CONTROL_FLAG_DYNAMICS_SATURATED;
    }
    else if (dynamics_limit < -1.0f)
    {
        dynamics_limit = -1.0f;
        flags |= BALANCE_CONTROL_FLAG_DYNAMICS_SATURATED;
    }
    lever_rad = asinf(dynamics_limit) -
        atan2f(input->car_accel_mps2, BALANCE_CONTROL_GRAVITY_MPS2);
    lever_deg = lever_rad * BALANCE_CONTROL_RAD_TO_DEG;

    angle_limit = (input->measurement_age_ms >
        control->config.fresh_measurement_ms) ?
        control->config.degraded_lever_angle_deg :
        control->config.max_lever_angle_deg;
    if (balance_control_abs(lever_deg) > angle_limit)
    {
        lever_deg = balance_control_clamp(lever_deg, -angle_limit,
                                          angle_limit);
        flags |= BALANCE_CONTROL_FLAG_ANGLE_SATURATED;
    }

    output->desired_ball_accel_mps2 = desired_accel;
    output->lever_angle_deg = lever_deg;
    output->flags = flags;
}

const balance_control_output_t *balance_control_get_output(
    const balance_control_t *control)
{
    return (NULL == control) ? NULL : &control->output;
}
