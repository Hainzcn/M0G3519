#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "ball_motion_profile.h"
#include "control_config.h"

typedef struct
{
    float elapsed_s;
    uint32 phase_changes;
    uint8 saw_accel;
    uint8 saw_cruise;
    uint8 saw_brake;
} run_result_t;

static ball_motion_profile_config_t make_production_config(void)
{
    ball_motion_profile_config_t config;

    config.drive_accel_mps2 = BALANCE_PROFILE_DRIVE_ACCEL_MPS2;
    config.brake_accel_mps2 = BALANCE_PROFILE_BRAKE_ACCEL_MPS2;
    config.max_velocity_mps = BALANCE_PROFILE_MAX_VELOCITY_MPS;
    config.brake_lookahead_s = BALANCE_PROFILE_BRAKE_LOOKAHEAD_S;
    config.position_tolerance_m = BALANCE_PROFILE_POSITION_TOLERANCE_M;
    config.velocity_tolerance_mps =
        BALANCE_PROFILE_VELOCITY_TOLERANCE_MPS;
    return config;
}

static run_result_t run_until_hold(ball_motion_profile_t *profile,
                                   uint32 max_steps,
                                   float direction)
{
    const float dt_s = (float)BALANCE_ESTIMATOR_PERIOD_MS * 0.001f;
    run_result_t result = {0};
    ball_motion_phase_enum previous_phase = profile->output.phase;
    float previous_position = profile->output.position_m;
    uint32 step;

    for (step = 0u; step < max_steps; step++)
    {
        ball_motion_profile_step(profile, dt_s);
        result.elapsed_s += dt_s;
        if (profile->output.phase != previous_phase)
        {
            result.phase_changes++;
            previous_phase = profile->output.phase;
        }
        if (BALL_MOTION_PHASE_ACCEL == profile->output.phase)
        {
            result.saw_accel = 1u;
        }
        if (BALL_MOTION_PHASE_CRUISE == profile->output.phase)
        {
            result.saw_cruise = 1u;
        }
        if (BALL_MOTION_PHASE_BRAKE == profile->output.phase)
        {
            result.saw_brake = 1u;
        }
        if (direction * (profile->output.position_m -
                         previous_position) < -0.000001f)
        {
            fprintf(stderr,
                    "non-monotonic profile at %.3fs: %.6f -> %.6f, phase %u\n",
                    result.elapsed_s, previous_position,
                    profile->output.position_m,
                    (unsigned int)profile->output.phase);
            assert(0 && "profile position changed away from target");
        }
        previous_position = profile->output.position_m;
        if (BALL_MOTION_PHASE_HOLD == profile->output.phase)
        {
            return result;
        }
    }
    assert(0 && "profile did not reach hold");
    return result;
}

int main(void)
{
    ball_motion_profile_t profile;
    ball_motion_profile_t baseline_profile;
    ball_motion_profile_config_t config = make_production_config();
    ball_motion_profile_config_t baseline_config = config;
    run_result_t positive;
    run_result_t negative;
    float lookahead_brake_s = 0.0f;
    float baseline_brake_s = 0.0f;
    uint32 step;

    baseline_config.brake_lookahead_s = 0.0f;
    ball_motion_profile_init(&baseline_profile, &baseline_config);
    ball_motion_profile_set_target(&baseline_profile, 0.050f);
    ball_motion_profile_step(
        &baseline_profile,
        (float)BALANCE_ESTIMATOR_PERIOD_MS * 0.001f);
    assert(baseline_profile.output.phase == BALL_MOTION_PHASE_ACCEL);
    assert(fabsf(baseline_profile.output.velocity_mps -
                  baseline_config.drive_accel_mps2 *
                  (float)BALANCE_ESTIMATOR_PERIOD_MS * 0.001f) < 0.000001f);

    ball_motion_profile_init(&profile, &config);
    ball_motion_profile_set_target(&profile, 0.050f);
    ball_motion_profile_init(&baseline_profile, &baseline_config);
    ball_motion_profile_set_target(&baseline_profile, 0.050f);
    for (step = 0u; step < 400u; step++)
    {
        const float dt_s =
            (float)BALANCE_ESTIMATOR_PERIOD_MS * 0.001f;

        ball_motion_profile_step(&profile, dt_s);
        ball_motion_profile_step(&baseline_profile, dt_s);
        if ((lookahead_brake_s == 0.0f) &&
            (BALL_MOTION_PHASE_BRAKE == profile.output.phase))
        {
            lookahead_brake_s = (float)(step + 1u) * dt_s;
        }
        if ((baseline_brake_s == 0.0f) &&
            (BALL_MOTION_PHASE_BRAKE == baseline_profile.output.phase))
        {
            baseline_brake_s = (float)(step + 1u) * dt_s;
        }
        if ((lookahead_brake_s > 0.0f) && (baseline_brake_s > 0.0f))
        {
            break;
        }
    }
    assert(lookahead_brake_s > 0.0f);
    assert(baseline_brake_s > lookahead_brake_s);
    assert(fabsf((baseline_brake_s - lookahead_brake_s) -
                 config.brake_lookahead_s) < 0.015f);

    ball_motion_profile_init(&profile, &config);
    ball_motion_profile_set_target(&profile, 0.050f);
    positive = run_until_hold(&profile, 400u, 1.0f);
    assert(positive.saw_accel);
    assert(positive.saw_cruise);
    assert(positive.saw_brake);
    assert(positive.phase_changes <= 4u);
    assert(positive.elapsed_s < 1.40f);
    assert(fabsf(profile.output.position_m - 0.050f) < 0.00001f);
    assert(fabsf(profile.output.velocity_mps) < 0.00001f);

    ball_motion_profile_set_target(&profile, -0.050f);
    negative = run_until_hold(&profile, 500u, -1.0f);
    assert(negative.saw_accel);
    assert(negative.saw_cruise);
    assert(negative.saw_brake);
    assert(negative.phase_changes <= 4u);
    assert(negative.elapsed_s < 2.10f);
    assert((positive.elapsed_s + negative.elapsed_s +
            (float)BALANCE_SEQUENCE_SETTLE_MS * 0.001f) < 3.60f);
    assert(fabsf(profile.output.position_m + 0.050f) < 0.00001f);
    assert(fabsf(profile.output.velocity_mps) < 0.00001f);

    ball_motion_profile_reset(&profile, 0.0f, 0.100f);
    ball_motion_profile_set_target(&profile, 0.0f);
    ball_motion_profile_step(
        &profile, (float)BALANCE_ESTIMATOR_PERIOD_MS * 0.001f);
    assert(profile.output.phase == BALL_MOTION_PHASE_BRAKE);
    assert(profile.output.velocity_mps > 0.0f);
    assert(profile.output.velocity_mps < 0.100f);

    puts("ball motion profile tests passed");
    return 0;
}
