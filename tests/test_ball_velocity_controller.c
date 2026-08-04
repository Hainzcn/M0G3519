#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ball_velocity_controller.h"

static void test_vehicle_feedforward_is_position_weighted(void)
{
    ball_velocity_controller_t controller;
    ball_velocity_controller_config_t config;
    ball_velocity_controller_input_t input;
    const ball_velocity_controller_output_t *output;

    memset(&config, 0, sizeof(config));
    config.max_target_beam_angle_deg = 5.0f;
    config.target_beam_angle_slew_deg_s = 1000.0f;
    config.beam_angle_kp_s_inv = 1.0f;
    config.max_beam_velocity_deg_s = 20.0f;
    config.vehicle_feedforward_position_cutoff_m = 0.040f;
    ball_velocity_controller_init(&controller, &config);

    memset(&input, 0, sizeof(input));
    input.observer_valid = 1u;
    input.control_dt_s = 0.02f;
    input.vehicle_feedforward_angle_deg = -2.0f;
    ball_velocity_controller_step(&controller, &input);
    output = ball_velocity_controller_get_output(&controller);
    assert(fabsf(output->vehicle_feedforward_scale - 1.0f) < 0.0001f);
    assert(fabsf(output->target_beam_angle_deg + 2.0f) < 0.0001f);

    input.position_m = 0.020f;
    ball_velocity_controller_step(&controller, &input);
    assert(fabsf(output->vehicle_feedforward_scale - 0.5f) < 0.0001f);
    assert(fabsf(output->target_beam_angle_deg + 1.0f) < 0.0001f);

    input.position_m = 0.040f;
    ball_velocity_controller_step(&controller, &input);
    assert(output->vehicle_feedforward_scale == 0.0f);
    assert(fabsf(output->target_beam_angle_deg) < 0.0001f);
}

int main(void)
{
    test_vehicle_feedforward_is_position_weighted();
    puts("ball velocity controller tests passed");
    return 0;
}
