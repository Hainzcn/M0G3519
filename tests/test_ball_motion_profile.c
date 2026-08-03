#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "ball_motion_profile.h"

static ball_motion_profile_config_t make_config(void)
{
    ball_motion_profile_config_t config;
    config.drive_accel_mps2 = 0.12f;
    config.brake_accel_mps2 = 0.16f;
    config.max_velocity_mps = 0.06f;
    config.max_jerk_mps3 = 2.5f;
    config.feedforward_lead_s = 0.02f;
    config.capture_position_m = 0.004f;
    config.capture_velocity_mps = 0.010f;
    config.position_tolerance_m = 0.0005f;
    config.velocity_tolerance_mps = 0.002f;
    return config;
}

static void test_profile(float target, uint8 expect_cruise)
{
    const float dt = 0.005f;
    ball_motion_profile_t profile;
    ball_motion_profile_config_t config = make_config();
    float previous_accel = 0.0f;
    float previous_position = 0.0f;
    uint8 saw_cruise = 0u;
    uint8 saw_lead_difference = 0u;
    uint32 step;

    ball_motion_profile_init(&profile, &config);
    ball_motion_profile_set_target(&profile, target);
    for (step = 0u; step < 1000u; step++)
    {
        ball_motion_profile_step(&profile, dt);
        assert(fabsf(profile.output.accel_mps2 - previous_accel) <=
               config.max_jerk_mps3 * dt + 0.00001f);
        assert(fabsf(profile.output.accel_mps2) <=
               config.brake_accel_mps2 + 0.00001f);
        assert(fabsf(profile.output.velocity_mps) <=
               config.max_velocity_mps + 0.00001f);
        assert(target * (profile.output.position_m - previous_position) >=
               -0.000001f);
        assert(target * (target - profile.output.position_m) >= -0.000001f);
        if (BALL_MOTION_PHASE_CRUISE == profile.output.phase) saw_cruise = 1u;
        if (fabsf(profile.output.feedforward_accel_mps2 -
                  profile.output.accel_mps2) > 0.0001f)
            saw_lead_difference = 1u;
        previous_accel = profile.output.accel_mps2;
        previous_position = profile.output.position_m;
        if ((BALL_MOTION_PHASE_CAPTURE == profile.output.phase) ||
            (BALL_MOTION_PHASE_HOLD == profile.output.phase)) break;
    }
    assert(step < 1000u);
    assert(fabsf(target - profile.output.position_m) <=
           config.capture_position_m + 0.001f);
    assert(fabsf(profile.output.velocity_mps) <=
           config.capture_velocity_mps + 0.001f);
    assert(saw_cruise == expect_cruise);
    assert(saw_lead_difference != 0u);
}

int main(void)
{
    ball_motion_profile_t profile;
    ball_motion_profile_config_t config = make_config();
    float previous_accel;

    test_profile(0.010f, 0u);
    test_profile(-0.010f, 0u);
    test_profile(0.050f, 1u);
    test_profile(0.200f, 1u);

    ball_motion_profile_init(&profile, &config);
    ball_motion_profile_set_target(&profile, 0.050f);
    for (uint32 step = 0u; step < 40u; step++)
        ball_motion_profile_step(&profile, 0.005f);
    ball_motion_profile_set_target(&profile, -0.050f);
    for (uint32 step = 0u; step < 1000u; step++)
    {
        ball_motion_profile_step(&profile, 0.005f);
        if ((BALL_MOTION_PHASE_CAPTURE == profile.output.phase) ||
            (BALL_MOTION_PHASE_HOLD == profile.output.phase)) break;
    }
    assert(profile.output.position_m < -0.045f);

    ball_motion_profile_reset(&profile, 0.075f, 0.0f);
    ball_motion_profile_set_target(&profile, 0.0f);
    previous_accel = profile.output.accel_mps2;
    for (uint32 step = 0u; step < 400u; step++)
    {
        if ((step % 4u) == 0u)
            ball_motion_profile_reanchor(&profile, 0.075f, 0.0f);
        ball_motion_profile_step(&profile, 0.005f);
        assert(fabsf(profile.output.accel_mps2 - previous_accel) <=
               config.max_jerk_mps3 * 0.005f + 0.00001f);
        previous_accel = profile.output.accel_mps2;
    }
    assert(profile.output.position_m > 0.070f);
    assert(profile.output.feedforward_accel_mps2 < 0.0f);
    assert(profile.output.phase != BALL_MOTION_PHASE_HOLD);

    puts("ball motion profile tests passed");
    return 0;
}
