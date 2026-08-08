#ifndef BALANCE_VELOCITY_ACTUATOR_H_
#define BALANCE_VELOCITY_ACTUATOR_H_

#include "zf_common_typedef.h"

#define BALANCE_VELOCITY_ACTUATOR_SOFT_LIMIT  (0x01u)
#define BALANCE_VELOCITY_ACTUATOR_HARD_LIMIT  (0x02u)
#define BALANCE_VELOCITY_ACTUATOR_QUANTIZED   (0x04u)
#define BALANCE_VELOCITY_ACTUATOR_NO_FEEDBACK (0x08u)

typedef struct
{
    float motor_sign;
    float raising_ratio;
    float lowering_ratio;
    float motor_min_soft_deg;
    float motor_max_soft_deg;
    float motor_min_hard_deg;
    float motor_max_hard_deg;
    int16 max_motor_rpm;
    int16 min_active_rpm;
} balance_velocity_actuator_config_t;

typedef struct
{
    float beam_velocity_deg_s;
    float motor_position_deg;
    uint8 motor_position_valid;
} balance_velocity_actuator_input_t;

typedef struct
{
    float requested_motor_rpm;
    float limited_motor_rpm;
    int16 command_motor_rpm;
    uint8 flags;
} balance_velocity_actuator_output_t;

typedef struct
{
    balance_velocity_actuator_config_t config;
    balance_velocity_actuator_output_t output;
} balance_velocity_actuator_t;

void balance_velocity_actuator_init(
    balance_velocity_actuator_t *actuator,
    const balance_velocity_actuator_config_t *config);
void balance_velocity_actuator_step(
    balance_velocity_actuator_t *actuator,
    const balance_velocity_actuator_input_t *input);
const balance_velocity_actuator_output_t *balance_velocity_actuator_get_output(
    const balance_velocity_actuator_t *actuator);

#endif
