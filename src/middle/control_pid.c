#include "control_pid.h"

static float control_pid_clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void control_pid_init(control_pid_t *pid,
                      const control_pid_config_t *config)
{
    if ((NULL == pid) || (NULL == config))
    {
        return;
    }

    pid->config = *config;
    control_pid_reset(pid);
}

void control_pid_reset(control_pid_t *pid)
{
    if (NULL == pid)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
    pid->initialized = 0u;
    pid->saturated = 0u;
}

float control_pid_step(control_pid_t *pid, float error, float feedforward,
                       float dt_s)
{
    float derivative = 0.0f;
    float integral_candidate;
    float unsaturated;
    float output;

    if ((NULL == pid) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    if (0u != pid->initialized)
    {
        derivative = (error - pid->previous_error) / dt_s;
    }
    else
    {
        pid->initialized = 1u;
    }

    integral_candidate = control_pid_clamp(
        pid->integral + error * dt_s, pid->config.integral_limit);
    unsaturated = feedforward + pid->config.kp * error +
                  pid->config.ki * integral_candidate +
                  pid->config.kd * derivative;
    output = control_pid_clamp(unsaturated, pid->config.output_limit);

    /* Integrate only when it does not drive an already saturated output out. */
    if ((output == unsaturated) ||
        ((output > 0.0f) && (error < 0.0f)) ||
        ((output < 0.0f) && (error > 0.0f)))
    {
        pid->integral = integral_candidate;
    }

    pid->previous_error = error;
    pid->output = output;
    pid->saturated = (output != unsaturated) ? 1u : 0u;
    return output;
}
