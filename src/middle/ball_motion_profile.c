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

static void ball_motion_reset_output(ball_motion_profile_output_t *output,
                                     float position_m,
                                     float velocity_mps)
{
    output->target_position_m = position_m;
    output->position_m = position_m;
    output->velocity_mps = velocity_mps;
    output->accel_mps2 = 0.0f;
    output->phase = BALL_MOTION_PHASE_HOLD;
}

static void ball_motion_step_output(
    const ball_motion_profile_config_t *config,
    ball_motion_profile_output_t *output,
    float dt_s)
{
    float error;
    float direction;
    float distance;
    float velocity_toward;
    float stop_distance;
    float old_velocity;

    error = output->target_position_m - output->position_m;
    distance = ball_motion_abs(error);

    if ((distance <= config->position_tolerance_m) &&
        (ball_motion_abs(output->velocity_mps) <=
         config->velocity_tolerance_mps))
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
            (2.0f * config->brake_accel_mps2) : 0.0f;

    if (velocity_toward < -config->velocity_tolerance_mps)
    {
        output->accel_mps2 = direction * config->brake_accel_mps2;
        output->phase = BALL_MOTION_PHASE_BRAKE;
    }
    else if ((velocity_toward > config->velocity_tolerance_mps) &&
             ((BALL_MOTION_PHASE_BRAKE == output->phase) ||
              (distance <= stop_distance)))
    {
        output->accel_mps2 = -direction * config->brake_accel_mps2;
        output->phase = BALL_MOTION_PHASE_BRAKE;
    }
    else if (velocity_toward >= config->max_velocity_mps)
    {
        output->accel_mps2 = 0.0f;
        output->phase = BALL_MOTION_PHASE_CRUISE;
    }
    else
    {
        output->accel_mps2 = direction * config->drive_accel_mps2;
        output->phase = BALL_MOTION_PHASE_ACCEL;
    }

    old_velocity = output->velocity_mps;
    output->velocity_mps += output->accel_mps2 * dt_s;
    output->velocity_mps = ball_motion_clamp(
        output->velocity_mps,
        -config->max_velocity_mps,
        config->max_velocity_mps);
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
    ball_motion_reset_output(&profile->nominal_output, position_m,
                             velocity_mps);
    profile->output = profile->nominal_output;
}

void ball_motion_profile_set_target(
    ball_motion_profile_t *profile,
    float target_position_m)
{
    if (NULL != profile)
    {
        /* Start a changed trajectory from the state currently commanded. */
        profile->nominal_output = profile->output;
        profile->nominal_output.target_position_m = target_position_m;
        profile->output.target_position_m = target_position_m;
    }
}

void ball_motion_profile_step(ball_motion_profile_t *profile, float dt_s)
{
    float lookahead_s;
    float prediction_step_s;

    if ((NULL == profile) || (dt_s <= 0.0f))
    {
        return;
    }
    ball_motion_step_output(&profile->config, &profile->nominal_output,
                            dt_s);
    profile->output = profile->nominal_output;

    lookahead_s = profile->config.brake_lookahead_s;
    while ((lookahead_s > 0.0f) &&
           (BALL_MOTION_PHASE_HOLD != profile->output.phase))
    {
        prediction_step_s = (lookahead_s < dt_s) ? lookahead_s : dt_s;
        ball_motion_step_output(&profile->config, &profile->output,
                                prediction_step_s);
        lookahead_s -= prediction_step_s;
    }
}

const ball_motion_profile_output_t *ball_motion_profile_get_output(
    const ball_motion_profile_t *profile)
{
    return (NULL == profile) ? NULL : &profile->output;
}
