#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_

#include "zf_common_typedef.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} control_pid_config_t;

typedef struct
{
    control_pid_config_t config;
    float integral;
    float previous_error;
    float feedforward;
    float feedback;
    float unsaturated_output;
    float output;
    uint8 initialized;
    uint8 saturated;
} control_pid_t;

void control_pid_init(control_pid_t *pid,
                      const control_pid_config_t *config);
void control_pid_reset(control_pid_t *pid);
float control_pid_step(control_pid_t *pid, float error, float feedforward,
                       float dt_s);

#endif
