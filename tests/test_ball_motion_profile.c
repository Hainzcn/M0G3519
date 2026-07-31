#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "ball_motion_profile.h"

static ball_motion_profile_config_t make_config(void)
{
    ball_motion_profile_config_t config;

    config.drive_accel_mps2 = 0.25f;
    config.brake_accel_mps2 = 0.25f;
    config.max_velocity_mps = 0.15f;
    config.brake_margin_m = 0.002f;
    config.position_tolerance_m = 0.0005f;
    config.velocity_tolerance_mps = 0.002f;
    return config;
}

static void run_until_hold(ball_motion_profile_t *profile,
                           uint32 max_steps,
                           uint8 *saw_accel,
                           uint8 *saw_brake)
{
    uint32 step;

    for (step = 0u; step < max_steps; step++)
    {
        ball_motion_profile_step(profile, 0.005f);
        if (BALL_MOTION_PHASE_ACCEL == profile->output.phase)
        {
            *saw_accel = 1u;
        }
        if (BALL_MOTION_PHASE_BRAKE == profile->output.phase)
        {
            *saw_brake = 1u;
        }
        if (BALL_MOTION_PHASE_HOLD == profile->output.phase)
        {
            return;
        }
    }
    assert(0 && "profile did not reach hold");
}

int main(void)
{
    ball_motion_profile_t profile;
    ball_motion_profile_config_t config = make_config();
    uint8 saw_accel = 0u;
    uint8 saw_brake = 0u;

    ball_motion_profile_init(&profile, &config);
    ball_motion_profile_set_target(&profile, 0.050f);
    run_until_hold(&profile, 600u, &saw_accel, &saw_brake);
    assert(saw_accel);
    assert(saw_brake);
    assert(fabsf(profile.output.position_m - 0.050f) < 0.00001f);
    assert(fabsf(profile.output.velocity_mps) < 0.00001f);

    saw_accel = 0u;
    saw_brake = 0u;
    ball_motion_profile_set_target(&profile, -0.050f);
    run_until_hold(&profile, 800u, &saw_accel, &saw_brake);
    assert(saw_accel);
    assert(saw_brake);
    assert(fabsf(profile.output.position_m + 0.050f) < 0.00001f);
    assert(fabsf(profile.output.velocity_mps) < 0.00001f);

    ball_motion_profile_reset(&profile, 0.0f, 0.100f);
    ball_motion_profile_set_target(&profile, 0.0f);
    ball_motion_profile_step(&profile, 0.005f);
    assert(profile.output.phase == BALL_MOTION_PHASE_BRAKE);
    assert(profile.output.velocity_mps > 0.0f);
    assert(profile.output.velocity_mps < 0.100f);

    puts("ball motion profile tests passed");
    return 0;
}
