#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "balance_control.h"
#include "balance_linkage.h"

static int near_value(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static balance_control_config_t make_config(void)
{
    balance_control_config_t config;

    config.kp = 8.0f;
    config.kd = 4.0f;
    config.position_correction_gain = 0.65f;
    config.velocity_correction_gain = 0.50f;
    config.reference_accel_gain = 0.50f;
    config.max_ball_accel_mps2 = 0.45f;
    config.edge_recovery_accel_mps2 = 0.22f;
    config.max_lever_angle_deg = 4.0f;
    config.degraded_lever_angle_deg = 2.0f;
    config.max_lever_rate_deg_s = 30.0f;
    config.edge_position_m = 0.100f;
    config.hard_edge_position_m = 0.115f;
    config.fresh_measurement_ms = 30u;
    config.valid_measurement_ms = 80u;
    return config;
}

static balance_control_input_t make_measurement(float position_m,
                                                float velocity_mps)
{
    balance_control_input_t input;

    input.new_measurement = 1u;
    input.measurement_valid = 1u;
    input.measured_position_m = position_m;
    input.measured_velocity_mps = velocity_mps;
    input.measurement_age_ms = 0u;
    input.reference_position_m = 0.0f;
    input.reference_velocity_mps = 0.0f;
    input.reference_accel_mps2 = 0.0f;
    input.car_accel_mps2 = 0.0f;
    input.actual_lever_valid = 0u;
    input.update_control_output = 1u;
    input.actual_lever_angle_deg = 0.0f;
    input.dt_s = 0.005f;
    return input;
}

int main(void)
{
    balance_control_t control;
    balance_control_config_t config = make_config();
    balance_control_input_t input;
    const balance_control_output_t *output;
    float motor_deg;
    float lever_deg;

    assert(balance_linkage_relative_motor_deg(0.0f, 5.0f, &motor_deg));
    assert(near_value(motor_deg, 22.96f, 0.10f));
    assert(balance_linkage_relative_motor_deg(0.0f, -5.0f, &motor_deg));
    assert(near_value(motor_deg, -18.23f, 0.10f));
    assert(balance_linkage_lever_from_relative_motor_deg(-5.0f,
                                                         17.625f,
                                                         &lever_deg));
    assert(near_value(lever_deg, 0.0f, 0.001f));
    assert(balance_linkage_lever_from_relative_motor_deg(-5.0f,
                                                         40.0f,
                                                         &lever_deg));
    assert(near_value(lever_deg, 5.0f, 0.001f));

    balance_control_init(&control, &config);
    input = make_measurement(0.050f, 0.0f);
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->lever_angle_deg > 0.0f);

    input.new_measurement = 0u;
    input.actual_lever_valid = 1u;
    input.actual_lever_angle_deg = 0.0f;
    input.update_control_output = 0u;
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(near_value(output->estimated_velocity_mps, 0.0f, 0.0001f));

    balance_control_reset(&control);
    input = make_measurement(-0.050f, 0.0f);
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->lever_angle_deg < 0.0f);

    balance_control_reset(&control);
    input = make_measurement(0.0f, 0.0f);
    input.reference_position_m = 0.010f;
    input.reference_velocity_mps = 0.020f;
    input.reference_accel_mps2 = 0.100f;
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(near_value(output->position_error_m, 0.010f, 0.0001f));
    assert(near_value(output->desired_ball_accel_mps2, 0.210f, 0.001f));

    balance_control_reset(&control);
    input = make_measurement(0.105f, 0.0f);
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->flags & BALANCE_CONTROL_FLAG_EDGE_RECOVERY);
    assert(near_value(output->desired_ball_accel_mps2, -0.22f, 0.001f));

    balance_control_reset(&control);
    input = make_measurement(-0.120f, 0.0f);
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->flags & BALANCE_CONTROL_FLAG_HARD_EDGE);
    assert(output->desired_ball_accel_mps2 > 0.0f);

    input.new_measurement = 0u;
    input.measurement_valid = 0u;
    input.measurement_age_ms = 81u;
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->flags & BALANCE_CONTROL_FLAG_PREDICT_ONLY);
    assert(near_value(output->desired_ball_accel_mps2, 0.0f, 0.0001f));
    assert(fabsf(output->lever_angle_deg) <= 2.0f);

    puts("balance control tests passed");
    return 0;
}
