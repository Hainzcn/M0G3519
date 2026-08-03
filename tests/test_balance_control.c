#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "balance_control.h"

static balance_control_config_t make_config(void)
{
    balance_control_config_t config;
    memset(&config, 0, sizeof(config));
    config.position_gain_s_inv = 1.0f;
    config.velocity_gain_s_inv = 5.0f;
    config.max_ball_velocity_mps = 0.060f;
    config.rolling_factor = 0.860739440f;
    config.rolling_friction_accel_mps2 = 0.100748322f;
    config.rail_curvature_m_inv = 0.0f;
    config.position_correction_gain = 0.65f;
    config.velocity_residual_gain = 0.65f;
    config.max_ball_accel_mps2 = 0.45f;
    config.brake_accel_mps2 = 0.35f;
    config.actuator_delay_s = 0.020f;
    config.brake_margin_delay_s = 0.020f;
    config.overspeed_release_ratio = 0.70f;
    config.overspeed_min_hold_ms = 40u;
    config.command_period_s = 0.020f;
    config.capture_position_m = 0.004f;
    config.center_dead_position_m = 0.002f;
    config.capture_velocity_mps = 0.010f;
    config.stick_velocity_mps = 0.005f;
    config.capture_integral_gain = 2.0f;
    config.capture_max_accel_mps2 = 0.05f;
    config.breakaway_angle_deg = 1.0f;
    config.breakaway_qualify_ms = 100u;
    config.breakaway_pulse_ms = 40u;
    config.breakaway_movement_m = 0.0006f;
    config.max_lever_angle_deg = 4.0f;
    config.degraded_lever_angle_deg = 2.0f;
    config.edge_recovery_accel_mps2 = 0.22f;
    config.edge_position_m = 0.100f;
    config.hard_edge_position_m = 0.125f;
    config.fresh_measurement_ms = 30u;
    config.valid_measurement_ms = 80u;
    config.calibration_provisional = 1u;
    return config;
}

static balance_control_input_t make_input(float position_m)
{
    balance_control_input_t input;
    memset(&input, 0, sizeof(input));
    input.new_measurement = 1u;
    input.measurement_valid = 1u;
    input.measured_position_m = position_m;
    input.measured_velocity_mps = 0.0f;
    input.measurement_interval_s = 0.020f;
    input.measurement_age_ms = 0u;
    input.target_position_m = 0.0f;
    input.reference_position_m = 0.0f;
    input.reference_holding = 1u;
    input.actuator_command_updated = 1u;
    input.actuator_command_angle_deg = 0.0f;
    input.update_control_output = 1u;
    input.dt_s = 0.005f;
    return input;
}

int main(void)
{
    balance_control_config_t config = make_config();
    balance_control_t control;
    balance_control_t left_control;
    balance_control_t right_control;
    balance_control_input_t input;
    balance_control_input_t left_input;
    balance_control_input_t right_input;
    const balance_control_output_t *output;

    balance_control_init(&control, &config);
    input = make_input(0.0f);
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->friction_mode == BALANCE_FRICTION_STOPPED);
    assert(fabsf(output->estimated_velocity_mps) < 0.0001f);
    assert(output->flags & BALANCE_CONTROL_FLAG_CALIBRATION_PENDING);
    assert(output->flags & BALANCE_CONTROL_FLAG_PREDICTOR_DEGRADED);

    balance_control_reset(&control);
    input = make_input(-0.003f);
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->friction_mode == BALANCE_FRICTION_CAPTURE);
    assert(output->flags & BALANCE_CONTROL_FLAG_CAPTURE_ACTIVE);
    assert(output->desired_ball_accel_mps2 > 0.0f);

    balance_control_reset(&control);
    input = make_input(-0.020f);
    input.measured_velocity_mps = 0.020f;
    for (uint8 step = 0u; step < 5u; step++)
    {
        balance_control_step(&control, &input);
    }
    output = balance_control_get_output(&control);
    assert(output->friction_mode == BALANCE_FRICTION_BREAKAWAY);
    assert(output->flags & BALANCE_CONTROL_FLAG_BREAKAWAY_ACTIVE);
    assert(fabsf(output->lever_angle_deg + 1.0f) < 0.001f);

    balance_control_reset(&control);
    input = make_input(-0.020f);
    for (uint8 step = 0u; step < 5u; step++)
    {
        input.measured_position_m += 0.0002f;
        balance_control_step(&control, &input);
    }
    output = balance_control_get_output(&control);
    assert(output->friction_mode != BALANCE_FRICTION_BREAKAWAY);

    balance_control_reset(&control);
    input = make_input(0.020f);
    input.measured_velocity_mps = 0.025f;
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(fabsf(output->estimated_velocity_mps - 0.025f) < 0.0001f);

    balance_control_reset(&control);
    input = make_input(0.010f);
    balance_control_step(&control, &input);
    control.output.estimated_velocity_mps = -0.100f;
    input.new_measurement = 0u;
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->flags & BALANCE_CONTROL_FLAG_OVERSPEED_PULLBACK);
    assert(output->desired_ball_accel_mps2 > 0.0f);

    balance_control_reset(&control);
    input = make_input(0.050f);
    balance_control_step(&control, &input);
    control.output.estimated_velocity_mps = 0.050f;
    input.new_measurement = 0u;
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->flags & BALANCE_CONTROL_FLAG_OVERSPEED_PULLBACK);
    assert(output->desired_ball_accel_mps2 < 0.0f);

    config.actuator_delay_s = 0.0f;
    balance_control_init(&control, &config);
    input = make_input(0.050f);
    input.measured_velocity_mps = -0.185f;
    balance_control_step(&control, &input);
    assert(control.overspeed_active != 0u);
    input.new_measurement = 0u;
    control.output.estimated_velocity_mps = -0.160f;
    balance_control_step(&control, &input);
    control.output.estimated_velocity_mps = -0.160f;
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->phase == BALANCE_CONTROL_PHASE_OVERSPEED);
    assert(control.overspeed_active != 0u);
    control.output.estimated_velocity_mps = -0.120f;
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->phase != BALANCE_CONTROL_PHASE_OVERSPEED);
    assert(control.overspeed_active == 0u);

    balance_control_reset(&control);
    input = make_input(0.050f);
    input.measured_velocity_mps = 0.010f;
    balance_control_step(&control, &input);
    assert(control.overspeed_active != 0u);
    input.new_measurement = 0u;
    control.output.estimated_velocity_mps = 0.0f;
    balance_control_step(&control, &input);
    control.output.estimated_velocity_mps = 0.0f;
    balance_control_step(&control, &input);
    assert(control.overspeed_active != 0u);
    control.output.estimated_velocity_mps = -0.006f;
    balance_control_step(&control, &input);
    assert(control.overspeed_active == 0u);

    balance_control_reset(&control);
    input = make_input(0.105f);
    balance_control_step(&control, &input);
    output = balance_control_get_output(&control);
    assert(output->flags & BALANCE_CONTROL_FLAG_EDGE_RECOVERY);
    assert(output->desired_ball_accel_mps2 < 0.0f);

    config.actuator_delay_s = 0.020f;
    balance_control_init(&control, &config);
    input = make_input(0.0f);
    balance_control_step(&control, &input);
    input.new_measurement = 0u;
    input.actuator_command_updated = 1u;
    input.actuator_command_angle_deg = 2.0f;
    for (uint8 step = 0u; step < 20u; step++)
    {
        input.update_control_output = 0u;
        balance_control_step(&control, &input);
    }
    output = balance_control_get_output(&control);
    assert(output->predicted_velocity_mps < output->estimated_velocity_mps);

    config.rail_curvature_m_inv = 0.201072373f;
    config.rolling_friction_accel_mps2 = 0.0f;
    balance_control_init(&left_control, &config);
    balance_control_init(&right_control, &config);
    left_input = make_input(-0.080f);
    right_input = make_input(0.080f);
    left_input.update_control_output = 0u;
    right_input.update_control_output = 0u;
    balance_control_step(&left_control, &left_input);
    balance_control_step(&right_control, &right_input);
    left_input.new_measurement = 0u;
    right_input.new_measurement = 0u;
    balance_control_step(&left_control, &left_input);
    balance_control_step(&right_control, &right_input);
    assert(left_control.output.estimated_velocity_mps > 0.0f);
    assert(right_control.output.estimated_velocity_mps < 0.0f);

    puts("balance control tests passed");
    return 0;
}
