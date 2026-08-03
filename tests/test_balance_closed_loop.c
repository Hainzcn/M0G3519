#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "balance_actuator_trajectory.h"
#include "balance_control.h"
#include "ball_motion_profile.h"

#define DT_S (0.005f)
#define OUTER_STEPS (4u)
#define GRAVITY_MPS2 (9.80665f)
#define ROLLING_FACTOR (0.704013961f)
#define ROLLING_FRICTION_MPS2 (0.081404074f)
#define RAIL_CURVATURE_M_INV (0.201072373f)

typedef struct
{
    balance_control_t control;
    ball_motion_profile_t profile;
    balance_actuator_trajectory_t actuator;
    float position_m;
    float velocity_mps;
    float delay_angles[4];
    uint8 command_pending;
} simulation_t;

static void simulation_init(simulation_t *simulation)
{
    balance_control_config_t control_config;
    ball_motion_profile_config_t profile_config;
    balance_actuator_trajectory_config_t actuator_config;
    memset(simulation, 0, sizeof(*simulation));
    memset(&control_config, 0, sizeof(control_config));
    control_config.position_gain_s_inv = 1.0f;
    control_config.velocity_gain_s_inv = 5.0f;
    control_config.max_ball_velocity_mps = 0.030f;
    control_config.rolling_factor = ROLLING_FACTOR;
    control_config.rolling_friction_accel_mps2 = ROLLING_FRICTION_MPS2;
    control_config.rail_curvature_m_inv = RAIL_CURVATURE_M_INV;
    control_config.position_correction_gain = 0.65f;
    control_config.velocity_residual_gain = 0.10f;
    control_config.max_ball_accel_mps2 = 0.45f;
    control_config.brake_accel_mps2 = 0.35f;
    control_config.actuator_delay_s = 0.020f;
    control_config.brake_margin_delay_s = 0.020f;
    control_config.command_period_s = 0.020f;
    control_config.capture_position_m = 0.004f;
    control_config.center_dead_position_m = 0.002f;
    control_config.capture_velocity_mps = 0.010f;
    control_config.stick_velocity_mps = 0.005f;
    control_config.capture_integral_gain = 2.0f;
    control_config.capture_max_accel_mps2 = 0.05f;
    control_config.breakaway_angle_deg = 1.0f;
    control_config.breakaway_qualify_ms = 100u;
    control_config.breakaway_pulse_ms = 40u;
    control_config.max_lever_angle_deg = 4.0f;
    control_config.degraded_lever_angle_deg = 2.0f;
    control_config.edge_recovery_accel_mps2 = 0.22f;
    control_config.edge_position_m = 0.100f;
    control_config.hard_edge_position_m = 0.125f;
    control_config.fresh_measurement_ms = 30u;
    control_config.valid_measurement_ms = 80u;
    balance_control_init(&simulation->control, &control_config);

    profile_config.drive_accel_mps2 = 0.12f;
    profile_config.brake_accel_mps2 = 0.16f;
    profile_config.max_velocity_mps = 0.030f;
    profile_config.max_jerk_mps3 = 2.5f;
    profile_config.feedforward_lead_s = 0.020f;
    profile_config.capture_position_m = 0.004f;
    profile_config.capture_velocity_mps = 0.010f;
    profile_config.position_tolerance_m = 0.0005f;
    profile_config.velocity_tolerance_mps = 0.002f;
    ball_motion_profile_init(&simulation->profile, &profile_config);

    actuator_config.max_angle_deg = 4.0f;
    actuator_config.max_rate_deg_s = 30.0f;
    actuator_config.max_accel_deg_s2 = 600.0f;
    balance_actuator_trajectory_init(&simulation->actuator, &actuator_config);
    simulation->command_pending = 1u;
}

static void simulation_step_plant(simulation_t *simulation)
{
    float angle_deg = simulation->delay_angles[0];
    float effective_angle_rad =
        angle_deg * 3.14159265358979323846f / 180.0f +
        RAIL_CURVATURE_M_INV * simulation->position_m;
    float drive_accel = -ROLLING_FACTOR * GRAVITY_MPS2 *
                        sinf(effective_angle_rad);
    float accel;
    for (uint8 index = 0u; index < 3u; index++)
        simulation->delay_angles[index] = simulation->delay_angles[index + 1u];
    simulation->delay_angles[3] = simulation->actuator.output.angle_deg;

    if (fabsf(simulation->velocity_mps) < 0.001f)
    {
        if (fabsf(drive_accel) <= ROLLING_FRICTION_MPS2)
        {
            simulation->velocity_mps = 0.0f;
            accel = 0.0f;
        }
        else
            accel = drive_accel - copysignf(ROLLING_FRICTION_MPS2, drive_accel);
    }
    else
        accel = drive_accel - copysignf(ROLLING_FRICTION_MPS2,
                                        simulation->velocity_mps);
    simulation->position_m += simulation->velocity_mps * DT_S +
                              0.5f * accel * DT_S * DT_S;
    simulation->velocity_mps += accel * DT_S;
}

static void run_leg(simulation_t *simulation, float target_m)
{
    balance_control_input_t input;
    const balance_control_output_t *control_output;
    float start_m = simulation->position_m;
    float direction = (target_m >= start_m) ? 1.0f : -1.0f;
    float previous_error = target_m - start_m;
    float max_overshoot = 0.0f;
    uint32 center_crossings = 0u;
    uint32 settled_steps = 0u;
    uint32 step;

    ball_motion_profile_reset(&simulation->profile, start_m,
                              simulation->velocity_mps);
    ball_motion_profile_set_target(&simulation->profile, target_m);
    for (step = 0u; step < 1200u; step++)
    {
        uint8 outer = ((step % OUTER_STEPS) == 0u) ? 1u : 0u;
        memset(&input, 0, sizeof(input));
        ball_motion_profile_step(&simulation->profile, DT_S);
        input.new_measurement = outer;
        input.measurement_valid = 1u;
        input.measured_position_m = simulation->position_m;
        input.measurement_interval_s = 0.020f;
        input.measurement_age_ms = 0u;
        input.target_position_m = target_m;
        input.reference_position_m = simulation->profile.output.position_m;
        input.reference_velocity_mps = simulation->profile.output.velocity_mps;
        input.feedforward_accel_mps2 =
            simulation->profile.output.feedforward_accel_mps2;
        input.reference_holding =
            (simulation->profile.output.phase == BALL_MOTION_PHASE_HOLD);
        input.actual_lever_valid = 1u;
        input.actual_lever_angle_deg = simulation->delay_angles[0];
        input.actuator_command_updated = simulation->command_pending;
        input.actuator_command_angle_deg = simulation->actuator.output.angle_deg;
        simulation->command_pending = 0u;
        input.update_control_output = outer;
        input.dt_s = DT_S;
        balance_control_step(&simulation->control, &input);
        control_output = balance_control_get_output(&simulation->control);
        if (outer)
        {
            balance_actuator_trajectory_step(&simulation->actuator,
                control_output->lever_angle_deg, 0.020f);
            simulation->command_pending = 1u;
        }
        simulation_step_plant(simulation);

        if ((previous_error * (target_m - simulation->position_m) < 0.0f) &&
            (fabsf(previous_error) > 0.0005f)) center_crossings++;
        previous_error = target_m - simulation->position_m;
        if (direction * (simulation->position_m - target_m) > max_overshoot)
            max_overshoot = direction * (simulation->position_m - target_m);
        if ((fabsf(previous_error) <= 0.004f) &&
            (fabsf(simulation->velocity_mps) <= 0.010f))
            settled_steps++;
        else
            settled_steps = 0u;
        if (settled_steps >= 40u) break;
    }
    if (step >= 1200u || max_overshoot > 0.005f || center_crossings > 1u)
        fprintf(stderr, "leg %.3f: step=%lu x=%.4f v=%.4f overshoot=%.4f crossings=%lu\n",
                target_m, (unsigned long)step, simulation->position_m,
                simulation->velocity_mps, max_overshoot,
                (unsigned long)center_crossings);
    assert(step < 1200u);
    assert(max_overshoot <= 0.005f);
    assert(center_crossings <= 1u);
}

int main(void)
{
    simulation_t simulation;
    simulation_init(&simulation);
    run_leg(&simulation, 0.030f);
    run_leg(&simulation, -0.030f);
    puts("balance closed-loop simulation passed");
    return 0;
}
