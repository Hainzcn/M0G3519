#include "ball_velocity_controller.h"

#include <math.h>
#include <string.h>

static float velocity_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float velocity_clamp(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

void ball_velocity_controller_init(
    ball_velocity_controller_t *controller,
    const ball_velocity_controller_config_t *config)
{
    if ((NULL == controller) || (NULL == config))
    {
        return;
    }
    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
}

void ball_velocity_controller_reset(ball_velocity_controller_t *controller)
{
    ball_velocity_controller_config_t config;

    if (NULL == controller)
    {
        return;
    }
    config = controller->config;
    memset(controller, 0, sizeof(*controller));
    controller->config = config;
}

void ball_velocity_controller_step(
    ball_velocity_controller_t *controller,
    const ball_velocity_controller_input_t *input)
{
    float error_m;
    float used_error_m;
    float proportional_velocity_mps;
    float unsigned_velocity_limit_mps;
    float braking_delay_velocity_mps;
    float near_scale = 1.0f;
    float feedforward_scale = 0.0f;
    float acceleration_mps2;
    float unrestricted_beam_angle_deg;
    float desired_beam_angle_deg;
    float target_angle_delta_deg;
    float max_target_angle_delta_deg;
    float corrected_angle_error_deg;
    float omega_deg_s;

    if ((NULL == controller) || (NULL == input))
    {
        return;
    }
    controller->output.flags = 0u;
    controller->output.beam_velocity_deg_s = 0.0f;
    controller->output.target_velocity_mps = 0.0f;
    controller->output.velocity_error_mps = 0.0f;
    controller->output.target_beam_angle_deg =
        controller->target_beam_angle_deg;
    controller->output.beam_angle_error_deg = 0.0f;
    error_m = input->position_m - input->target_position_m;
    controller->output.position_error_m = error_m;

    if (0u == input->observer_valid)
    {
        controller->position_active = 0u;
        controller->output.integral_velocity_mps = 0.0f;
        controller->has_previous_velocity = 0u;
        return;
    }

    if ((0u == controller->position_active) &&
        (velocity_abs(error_m) >= controller->config.position_on_m))
    {
        controller->position_active = 1u;
    }
    else if ((0u != controller->position_active) &&
             (velocity_abs(error_m) <= controller->config.position_off_m))
    {
        controller->position_active = 0u;
        controller->output.integral_velocity_mps = 0.0f;
    }
    used_error_m = (0u != controller->position_active) ? error_m : 0.0f;
    if (0u != controller->position_active)
    {
        controller->output.flags |= BALL_VELOCITY_CONTROL_POSITION_ACTIVE;
    }

    braking_delay_velocity_mps =
        controller->config.braking_envelope_mps2 *
        controller->config.actuator_delay_s;
    unsigned_velocity_limit_mps = sqrtf(
        braking_delay_velocity_mps * braking_delay_velocity_mps +
        2.0f * controller->config.braking_envelope_mps2 *
            velocity_abs(used_error_m)) - braking_delay_velocity_mps;
    if (unsigned_velocity_limit_mps >
        controller->config.max_target_velocity_mps)
    {
        unsigned_velocity_limit_mps =
            controller->config.max_target_velocity_mps;
    }
    controller->output.velocity_limit_mps = unsigned_velocity_limit_mps;

    if ((controller->config.position_ki_s2_inv != 0.0f) &&
        (0u != input->new_measurement) &&
        (0u != controller->position_active) &&
        (0u == input->output_saturated) &&
        (0u == input->freeze_integral) &&
        (velocity_abs(error_m) < controller->config.integral_zone_m) &&
        (input->measurement_dt_s > 0.0f))
    {
        controller->output.integral_velocity_mps +=
            controller->config.position_ki_s2_inv * error_m *
            input->measurement_dt_s;
        controller->output.integral_velocity_mps =
            velocity_clamp(controller->output.integral_velocity_mps,
                -controller->config.integral_velocity_limit_mps,
                controller->config.integral_velocity_limit_mps);
        controller->output.flags |= BALL_VELOCITY_CONTROL_INTEGRAL_ACTIVE;
    }

    proportional_velocity_mps =
        controller->config.position_kp_s_inv * used_error_m +
        controller->output.integral_velocity_mps;
    controller->output.target_velocity_mps = -velocity_clamp(
        proportional_velocity_mps,
        -unsigned_velocity_limit_mps,
        unsigned_velocity_limit_mps);
    if (velocity_abs(proportional_velocity_mps) >
        unsigned_velocity_limit_mps)
    {
        controller->output.flags |= BALL_VELOCITY_CONTROL_VELOCITY_LIMITED;
    }

    if ((controller->config.near_gain > 0.0f) &&
        (controller->config.near_position_m > 0.0f) &&
        ((error_m * input->velocity_mps) < 0.0f) &&
        (velocity_abs(error_m) < controller->config.near_position_m))
    {
        near_scale = 1.0f + controller->config.near_gain *
            (1.0f - velocity_abs(error_m) /
             controller->config.near_position_m);
        near_scale = velocity_clamp(
            near_scale, 1.0f, controller->config.near_scale_max);
        controller->output.flags |= BALL_VELOCITY_CONTROL_NEAR_DAMPING;
    }
    controller->output.effective_kv_deg_per_mm =
        controller->config.velocity_kv_deg_per_mmps * near_scale;
    controller->output.velocity_error_mps =
        input->velocity_mps - controller->output.target_velocity_mps;

    if (controller->config.vehicle_feedforward_position_cutoff_m > 0.0f)
    {
        feedforward_scale = velocity_clamp(
            1.0f - velocity_abs(error_m) /
                controller->config.vehicle_feedforward_position_cutoff_m,
            0.0f, 1.0f);
    }
    controller->output.vehicle_feedforward_scale = feedforward_scale;
    controller->output.vehicle_feedforward_angle_deg =
        input->vehicle_feedforward_angle_deg * feedforward_scale;

    if ((0u != input->new_measurement) &&
        (input->measurement_dt_s > 0.0f))
    {
        if (0u != controller->has_previous_velocity)
        {
            acceleration_mps2 =
                (input->velocity_mps - controller->previous_velocity_mps) /
                input->measurement_dt_s;
            controller->output.filtered_acceleration_mps2 +=
                controller->config.acceleration_filter_alpha *
                (acceleration_mps2 -
                 controller->output.filtered_acceleration_mps2);
        }
        controller->previous_velocity_mps = input->velocity_mps;
        controller->has_previous_velocity = 1u;
    }
    unrestricted_beam_angle_deg =
        controller->output.effective_kv_deg_per_mm *
            controller->output.velocity_error_mps * 1000.0f +
        controller->config.acceleration_ka_deg_per_mps2 *
            controller->output.filtered_acceleration_mps2 +
        controller->config.fixed_beam_bias_deg +
        controller->output.vehicle_feedforward_angle_deg;
    desired_beam_angle_deg = velocity_clamp(
        unrestricted_beam_angle_deg,
        -controller->config.max_target_beam_angle_deg,
        controller->config.max_target_beam_angle_deg);
    if (desired_beam_angle_deg != unrestricted_beam_angle_deg)
    {
        controller->output.flags |= BALL_VELOCITY_CONTROL_ANGLE_LIMITED;
    }

    /* Slew the attitude reference, then close the loop on measured beam angle. */
    if (0u == controller->target_beam_angle_initialized)
    {
        controller->target_beam_angle_deg = input->measured_beam_angle_deg;
        controller->target_beam_angle_initialized = 1u;
    }
    max_target_angle_delta_deg =
        controller->config.target_beam_angle_slew_deg_s * input->control_dt_s;
    target_angle_delta_deg =
        desired_beam_angle_deg - controller->target_beam_angle_deg;
    target_angle_delta_deg = velocity_clamp(
        target_angle_delta_deg,
        -max_target_angle_delta_deg,
        max_target_angle_delta_deg);
    if ((controller->target_beam_angle_deg + target_angle_delta_deg) !=
        desired_beam_angle_deg)
    {
        controller->output.flags |= BALL_VELOCITY_CONTROL_ANGLE_SLEWED;
    }
    controller->target_beam_angle_deg += target_angle_delta_deg;
    controller->target_beam_angle_deg = velocity_clamp(
        controller->target_beam_angle_deg,
        -controller->config.max_target_beam_angle_deg,
        controller->config.max_target_beam_angle_deg);
    controller->output.target_beam_angle_deg =
        controller->target_beam_angle_deg;
    controller->output.beam_angle_error_deg =
        controller->target_beam_angle_deg - input->measured_beam_angle_deg;
    if (velocity_abs(controller->output.beam_angle_error_deg) <=
        controller->config.beam_angle_deadband_deg)
    {
        omega_deg_s = 0.0f;
    }
    else
    {
        corrected_angle_error_deg =
            controller->output.beam_angle_error_deg -
            ((controller->output.beam_angle_error_deg > 0.0f) ?
                controller->config.beam_angle_deadband_deg :
                -controller->config.beam_angle_deadband_deg);
        omega_deg_s = controller->config.beam_angle_kp_s_inv *
                      corrected_angle_error_deg;
    }
    controller->output.beam_velocity_deg_s = velocity_clamp(
        omega_deg_s,
        -controller->config.max_beam_velocity_deg_s,
        controller->config.max_beam_velocity_deg_s);
    if (omega_deg_s != controller->output.beam_velocity_deg_s)
    {
        controller->output.flags |= BALL_VELOCITY_CONTROL_OMEGA_LIMITED;
    }
}

const ball_velocity_controller_output_t *ball_velocity_controller_get_output(
    const ball_velocity_controller_t *controller)
{
    return (NULL == controller) ? NULL : &controller->output;
}
