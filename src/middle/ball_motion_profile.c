#include "ball_motion_profile.h"

static float ball_motion_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ball_motion_clamp(float value, float low, float high)
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

void ball_motion_profile_init(
    ball_motion_profile_t *profile,
    const ball_motion_profile_config_t *config)
{
    if ((NULL == profile) || (NULL == config))
    {
        return;
    }
    profile->config = *config;
    ball_motion_profile_reset(profile, 0.0f, 0.0f);
}

void ball_motion_profile_reset(
    ball_motion_profile_t *profile,
    float position_m,
    float velocity_mps)
{
    if (NULL == profile)
    {
        return;
    }
    profile->output.target_position_m = position_m;
    profile->output.position_m = position_m;
    profile->output.velocity_mps = velocity_mps;
    profile->output.accel_mps2 = 0.0f;
    profile->output.phase = BALL_MOTION_PHASE_HOLD;
}

void ball_motion_profile_set_target(
    ball_motion_profile_t *profile,
    float target_position_m)
{
    if (NULL != profile)
    {
        profile->output.target_position_m = target_position_m;
    }
}

void ball_motion_profile_step(ball_motion_profile_t *profile, float dt_s)
{
    ball_motion_profile_output_t *output;
    float error;
    float direction;
    float distance;
    float velocity_toward;
    float stop_distance;
    float old_velocity;

    if ((NULL == profile) || (dt_s <= 0.0f))
    {
        return;
    }
    output = &profile->output;
    error = output->target_position_m - output->position_m;
    distance = ball_motion_abs(error);

    if ((distance <= profile->config.position_tolerance_m) &&
        (ball_motion_abs(output->velocity_mps) <=
         profile->config.velocity_tolerance_mps))
    {
        output->position_m = output->target_position_m;
        output->velocity_mps = 0.0f;
        output->accel_mps2 = 0.0f;
        output->phase = BALL_MOTION_PHASE_HOLD;
        return;
    }

    direction = (error >= 0.0f) ? 1.0f : -1.0f;
    velocity_toward = direction * output->velocity_mps;
    stop_distance = (velocity_toward > 0.0f) ?
        (velocity_toward * velocity_toward) /
            (2.0f * profile->config.brake_accel_mps2) : 0.0f;

    if (velocity_toward < -profile->config.velocity_tolerance_mps)
    {
        output->accel_mps2 = direction * profile->config.brake_accel_mps2;
        output->phase = BALL_MOTION_PHASE_BRAKE;
    }
    else if ((velocity_toward > profile->config.velocity_tolerance_mps) &&
             (distance <= stop_distance +
              profile->config.brake_margin_m))
    {
        output->accel_mps2 = -direction * profile->config.brake_accel_mps2;
        output->phase = BALL_MOTION_PHASE_BRAKE;
    }
    else if (velocity_toward >= profile->config.max_velocity_mps)
    {
        output->accel_mps2 = 0.0f;
        output->phase = BALL_MOTION_PHASE_CRUISE;
    }
    else
    {
        output->accel_mps2 = direction * profile->config.drive_accel_mps2;
        output->phase = BALL_MOTION_PHASE_ACCEL;
    }

    old_velocity = output->velocity_mps;
    output->velocity_mps += output->accel_mps2 * dt_s;
    output->velocity_mps = ball_motion_clamp(
        output->velocity_mps,
        -profile->config.max_velocity_mps,
        profile->config.max_velocity_mps);
    if ((BALL_MOTION_PHASE_BRAKE == output->phase) &&
        (old_velocity * output->velocity_mps < 0.0f))
    {
        output->velocity_mps = 0.0f;
    }
    output->position_m +=
        0.5f * (old_velocity + output->velocity_mps) * dt_s;

    if (((error > 0.0f) &&
         (output->position_m >= output->target_position_m)) ||
        ((error < 0.0f) &&
         (output->position_m <= output->target_position_m)))
    {
        output->position_m = output->target_position_m;
        output->velocity_mps = 0.0f;
        output->accel_mps2 = 0.0f;
        output->phase = BALL_MOTION_PHASE_HOLD;
    }
}

const ball_motion_profile_output_t *ball_motion_profile_get_output(
    const ball_motion_profile_t *profile)
{
    return (NULL == profile) ? NULL : &profile->output;
}
