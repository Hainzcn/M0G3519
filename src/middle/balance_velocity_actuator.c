#include "balance_velocity_actuator.h"

#include <string.h>

static float actuator_clamp(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

void balance_velocity_actuator_init(
    balance_velocity_actuator_t *actuator,
    const balance_velocity_actuator_config_t *config)
{
    if ((NULL == actuator) || (NULL == config))
    {
        return;
    }
    memset(actuator, 0, sizeof(*actuator));
    actuator->config = *config;
}

void balance_velocity_actuator_step(
    balance_velocity_actuator_t *actuator,
    const balance_velocity_actuator_input_t *input)
{
    float ratio;
    float rpm;
    float scale;
    int16 quantized_rpm;

    if ((NULL == actuator) || (NULL == input))
    {
        return;
    }
    memset(&actuator->output, 0, sizeof(actuator->output));
    if (0u == input->motor_position_valid)
    {
        actuator->output.flags = BALANCE_VELOCITY_ACTUATOR_NO_FEEDBACK;
        return;
    }
    ratio = (input->beam_velocity_deg_s >= 0.0f) ?
        actuator->config.raising_ratio : actuator->config.lowering_ratio;
    rpm = actuator->config.motor_sign *
          input->beam_velocity_deg_s * ratio / 6.0f;
    actuator->output.requested_motor_rpm = rpm;
    if (((input->motor_position_deg <=
          actuator->config.motor_min_hard_deg) && (rpm < 0.0f)) ||
        ((input->motor_position_deg >=
          actuator->config.motor_max_hard_deg) && (rpm > 0.0f)))
    {
        actuator->output.flags = BALANCE_VELOCITY_ACTUATOR_HARD_LIMIT;
        return;
    }
    if ((actuator->config.motor_min_soft_deg >
         actuator->config.motor_min_hard_deg) &&
        (rpm < 0.0f) &&
        (input->motor_position_deg < actuator->config.motor_min_soft_deg))
    {
        scale = (input->motor_position_deg -
                 actuator->config.motor_min_hard_deg) /
                (actuator->config.motor_min_soft_deg -
                 actuator->config.motor_min_hard_deg);
        rpm *= actuator_clamp(scale, 0.0f, 1.0f);
        actuator->output.flags |= BALANCE_VELOCITY_ACTUATOR_SOFT_LIMIT;
    }
    else if ((actuator->config.motor_max_soft_deg <
              actuator->config.motor_max_hard_deg) &&
             (rpm > 0.0f) &&
             (input->motor_position_deg >
              actuator->config.motor_max_soft_deg))
    {
        scale = (actuator->config.motor_max_hard_deg -
                 input->motor_position_deg) /
                (actuator->config.motor_max_hard_deg -
                 actuator->config.motor_max_soft_deg);
        rpm *= actuator_clamp(scale, 0.0f, 1.0f);
        actuator->output.flags |= BALANCE_VELOCITY_ACTUATOR_SOFT_LIMIT;
    }
    rpm = actuator_clamp(rpm,
        -(float)actuator->config.max_motor_rpm,
        (float)actuator->config.max_motor_rpm);
    actuator->output.limited_motor_rpm = rpm;
    quantized_rpm = (int16)(rpm + ((rpm >= 0.0f) ? 0.5f : -0.5f));
    if ((0 == quantized_rpm) && (0.0f != rpm))
    {
        quantized_rpm = (rpm > 0.0f) ? actuator->config.min_active_rpm :
                                      -actuator->config.min_active_rpm;
    }
    actuator->output.command_motor_rpm = quantized_rpm;
    if ((float)quantized_rpm != rpm)
    {
        actuator->output.flags |= BALANCE_VELOCITY_ACTUATOR_QUANTIZED;
    }
}

const balance_velocity_actuator_output_t *balance_velocity_actuator_get_output(
    const balance_velocity_actuator_t *actuator)
{
    return (NULL == actuator) ? NULL : &actuator->output;
}
