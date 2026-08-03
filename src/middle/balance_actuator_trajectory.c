#include "balance_actuator_trajectory.h"

#include <math.h>

static float trajectory_clamp(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

void balance_actuator_trajectory_init(
    balance_actuator_trajectory_t *trajectory,
    const balance_actuator_trajectory_config_t *config)
{
    if ((NULL == trajectory) || (NULL == config)) return;
    trajectory->config = *config;
    balance_actuator_trajectory_reset(trajectory, 0.0f);
}

void balance_actuator_trajectory_reset(
    balance_actuator_trajectory_t *trajectory, float angle_deg)
{
    if (NULL == trajectory) return;
    trajectory->output.angle_deg = trajectory_clamp(
        angle_deg, -trajectory->config.max_angle_deg,
        trajectory->config.max_angle_deg);
    trajectory->output.rate_deg_s = 0.0f;
    trajectory->output.saturated = 0u;
}

void balance_actuator_trajectory_step(
    balance_actuator_trajectory_t *trajectory, float target_angle_deg,
    float dt_s)
{
    float target;
    float error;
    float stopping_rate;
    float desired_rate;
    float rate_delta;
    float next_angle;

    if ((NULL == trajectory) || (dt_s <= 0.0f)) return;
    target = trajectory_clamp(target_angle_deg,
        -trajectory->config.max_angle_deg,
        trajectory->config.max_angle_deg);
    error = target - trajectory->output.angle_deg;
    stopping_rate = sqrtf(2.0f * trajectory->config.max_accel_deg_s2 *
                          ((error < 0.0f) ? -error : error)) -
                    0.5f * trajectory->config.max_accel_deg_s2 * dt_s;
    if (stopping_rate <= 0.0f)
        stopping_rate = ((error < 0.0f) ? -error : error) / dt_s;
    desired_rate = (error < 0.0f) ? -stopping_rate : stopping_rate;
    desired_rate = trajectory_clamp(desired_rate,
        -trajectory->config.max_rate_deg_s,
        trajectory->config.max_rate_deg_s);
    rate_delta = trajectory_clamp(
        desired_rate - trajectory->output.rate_deg_s,
        -trajectory->config.max_accel_deg_s2 * dt_s,
        trajectory->config.max_accel_deg_s2 * dt_s);
    trajectory->output.saturated =
        ((target != target_angle_deg) ||
         (fabsf(desired_rate - trajectory->output.rate_deg_s) >
          fabsf(rate_delta) + 0.0001f)) ? 1u : 0u;
    trajectory->output.rate_deg_s += rate_delta;
    next_angle = trajectory->output.angle_deg +
                 trajectory->output.rate_deg_s * dt_s;
    if (((error >= 0.0f) && (next_angle >= target)) ||
        ((error < 0.0f) && (next_angle <= target)))
    {
        next_angle = target;
        trajectory->output.rate_deg_s = 0.0f;
    }
    trajectory->output.angle_deg = next_angle;
}

const balance_actuator_trajectory_output_t *
balance_actuator_trajectory_get_output(
    const balance_actuator_trajectory_t *trajectory)
{
    return (NULL == trajectory) ? NULL : &trajectory->output;
}
