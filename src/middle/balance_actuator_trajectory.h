#ifndef BALANCE_ACTUATOR_TRAJECTORY_H_
#define BALANCE_ACTUATOR_TRAJECTORY_H_

#include "zf_common_typedef.h"

typedef struct
{
    float max_angle_deg;
    float max_rate_deg_s;
    float max_accel_deg_s2;
} balance_actuator_trajectory_config_t;

typedef struct
{
    float angle_deg;
    float rate_deg_s;
    uint8 saturated;
} balance_actuator_trajectory_output_t;

typedef struct
{
    balance_actuator_trajectory_config_t config;
    balance_actuator_trajectory_output_t output;
} balance_actuator_trajectory_t;

void balance_actuator_trajectory_init(
    balance_actuator_trajectory_t *trajectory,
    const balance_actuator_trajectory_config_t *config);
void balance_actuator_trajectory_reset(
    balance_actuator_trajectory_t *trajectory, float angle_deg);
void balance_actuator_trajectory_step(
    balance_actuator_trajectory_t *trajectory, float target_angle_deg,
    float dt_s);
const balance_actuator_trajectory_output_t *
balance_actuator_trajectory_get_output(
    const balance_actuator_trajectory_t *trajectory);

#endif
