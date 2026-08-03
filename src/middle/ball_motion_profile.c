#include "ball_motion_profile.h"

#include <math.h>

static float profile_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float profile_clamp(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

static float profile_move_toward(float value, float target, float delta)
{
    if (value < target) return (value + delta > target) ? target : value + delta;
    return (value - delta < target) ? target : value - delta;
}

static void profile_reset_output(ball_motion_profile_output_t *output,
                                 float position_m, float velocity_mps)
{
    output->target_position_m = position_m;
    output->position_m = position_m;
    output->velocity_mps = velocity_mps;
    output->accel_mps2 = 0.0f;
    output->feedforward_accel_mps2 = 0.0f;
    output->phase = BALL_MOTION_PHASE_HOLD;
}

static void profile_step_output(const ball_motion_profile_config_t *config,
                                ball_motion_profile_output_t *output,
                                float dt_s)
{
    float error = output->target_position_m - output->position_m;
    float distance = profile_abs(error);
    float direction = (error >= 0.0f) ? 1.0f : -1.0f;
    float velocity_toward = direction * output->velocity_mps;
    float available = distance - config->capture_position_m;
    float brake_velocity;
    float velocity_excess;
    float brake_distance;
    float desired_accel;
    float old_accel;
    float old_velocity;

    if ((distance <= config->position_tolerance_m) &&
        (profile_abs(output->velocity_mps) <= config->velocity_tolerance_mps))
    {
        output->position_m = output->target_position_m;
        output->velocity_mps = 0.0f;
        output->accel_mps2 = 0.0f;
        output->phase = BALL_MOTION_PHASE_HOLD;
        return;
    }

    available = (available > 0.0f) ? available : 0.0f;
    brake_velocity = config->capture_velocity_mps +
        sqrtf(2.0f * config->brake_accel_mps2 * available);
    velocity_excess = velocity_toward - config->capture_velocity_mps;
    if (velocity_excess < 0.0f) velocity_excess = 0.0f;
    brake_distance = velocity_excess * velocity_excess /
        (2.0f * config->brake_accel_mps2);
    if (velocity_toward > 0.0f)
    {
        brake_distance += velocity_toward *
            (profile_abs(output->accel_mps2) + config->brake_accel_mps2) /
            config->max_jerk_mps3;
    }
    if ((distance <= config->capture_position_m) &&
        (velocity_toward >= 0.0f) &&
        (velocity_toward <= config->capture_velocity_mps +
                            config->velocity_tolerance_mps))
    {
        desired_accel = 0.0f;
        output->phase = BALL_MOTION_PHASE_BRAKE;
    }
    else if (velocity_toward < -config->velocity_tolerance_mps)
    {
        desired_accel = direction * config->brake_accel_mps2;
        output->phase = BALL_MOTION_PHASE_BRAKE;
    }
    else if ((velocity_toward >= brake_velocity) ||
             (brake_distance >= available))
    {
        desired_accel = -direction * config->brake_accel_mps2;
        output->phase = BALL_MOTION_PHASE_BRAKE;
    }
    else if (velocity_toward >= config->max_velocity_mps)
    {
        desired_accel = 0.0f;
        output->phase = BALL_MOTION_PHASE_CRUISE;
    }
    else
    {
        desired_accel = direction * config->drive_accel_mps2;
        output->phase = BALL_MOTION_PHASE_ACCEL;
    }

    old_accel = output->accel_mps2;
    output->accel_mps2 = profile_move_toward(
        old_accel, desired_accel, config->max_jerk_mps3 * dt_s);
    old_velocity = output->velocity_mps;
    output->velocity_mps +=
        0.5f * (old_accel + output->accel_mps2) * dt_s;
    output->velocity_mps = profile_clamp(output->velocity_mps,
        -config->max_velocity_mps, config->max_velocity_mps);
    output->position_m +=
        0.5f * (old_velocity + output->velocity_mps) * dt_s;

    if ((distance <= config->capture_position_m) &&
        (direction * output->velocity_mps >= 0.0f) &&
        (direction * output->velocity_mps <=
         config->capture_velocity_mps + config->velocity_tolerance_mps) &&
        (profile_abs(output->accel_mps2) < 0.000001f))
    {
        output->phase = BALL_MOTION_PHASE_CAPTURE;
        return;
    }

    if (((error > 0.0f) && (output->position_m >= output->target_position_m)) ||
        ((error < 0.0f) && (output->position_m <= output->target_position_m)))
    {
        output->position_m = output->target_position_m;
        output->velocity_mps = 0.0f;
        output->accel_mps2 = 0.0f;
        output->phase = BALL_MOTION_PHASE_HOLD;
    }
}

void ball_motion_profile_init(ball_motion_profile_t *profile,
                              const ball_motion_profile_config_t *config)
{
    if ((NULL == profile) || (NULL == config)) return;
    profile->config = *config;
    ball_motion_profile_reset(profile, 0.0f, 0.0f);
}

void ball_motion_profile_reset(ball_motion_profile_t *profile,
                               float position_m, float velocity_mps)
{
    if (NULL == profile) return;
    profile_reset_output(&profile->nominal_output, position_m, velocity_mps);
    profile->output = profile->nominal_output;
}

void ball_motion_profile_set_target(ball_motion_profile_t *profile,
                                    float target_position_m)
{
    if (NULL == profile) return;
    profile->nominal_output = profile->output;
    profile->nominal_output.target_position_m = target_position_m;
    profile->output.target_position_m = target_position_m;
}

void ball_motion_profile_step(ball_motion_profile_t *profile, float dt_s)
{
    ball_motion_profile_output_t lead;
    float remaining;
    float step_s;

    if ((NULL == profile) || (dt_s <= 0.0f)) return;
    profile_step_output(&profile->config, &profile->nominal_output, dt_s);
    profile->output = profile->nominal_output;
    lead = profile->nominal_output;
    remaining = profile->config.feedforward_lead_s;
    while ((remaining > 0.0f) &&
           (BALL_MOTION_PHASE_HOLD != lead.phase) &&
           (BALL_MOTION_PHASE_CAPTURE != lead.phase))
    {
        step_s = (remaining < dt_s) ? remaining : dt_s;
        profile_step_output(&profile->config, &lead, step_s);
        remaining -= step_s;
    }
    profile->output.feedforward_accel_mps2 = lead.accel_mps2;
}

const ball_motion_profile_output_t *ball_motion_profile_get_output(
    const ball_motion_profile_t *profile)
{
    return (NULL == profile) ? NULL : &profile->output;
}
