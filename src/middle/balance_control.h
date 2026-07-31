#ifndef BALANCE_CONTROL_H_
#define BALANCE_CONTROL_H_

#include "zf_common_typedef.h"

#define BALANCE_CONTROL_FLAG_MEASUREMENT_FRESH  (0x01u)
#define BALANCE_CONTROL_FLAG_PREDICT_ONLY       (0x02u)
#define BALANCE_CONTROL_FLAG_EDGE_RECOVERY      (0x04u)
#define BALANCE_CONTROL_FLAG_DYNAMICS_SATURATED (0x08u)
#define BALANCE_CONTROL_FLAG_ANGLE_SATURATED    (0x10u)
#define BALANCE_CONTROL_FLAG_SLEW_SATURATED     (0x20u)
#define BALANCE_CONTROL_FLAG_HARD_EDGE          (0x40u)

typedef struct
{
    float kp;
    float kd;
    float position_correction_gain;
    float velocity_correction_gain;
    float max_ball_accel_mps2;
    float max_lever_angle_deg;
    float degraded_lever_angle_deg;
    float max_lever_rate_deg_s;
    float edge_position_m;
    float hard_edge_position_m;
    uint32 fresh_measurement_ms;
    uint32 valid_measurement_ms;
} balance_control_config_t;

typedef struct
{
    uint8 new_measurement;
    uint8 measurement_valid;
    float measured_position_m;
    float measured_velocity_mps;
    uint32 measurement_age_ms;
    float car_accel_mps2;
    uint8 actual_lever_valid;
    uint8 update_control_output;
    float actual_lever_angle_deg;
    float dt_s;
} balance_control_input_t;

typedef struct
{
    uint8 has_state;
    uint8 flags;
    float estimated_position_m;
    float estimated_velocity_mps;
    float position_error_m;
    float desired_ball_accel_mps2;
    float lever_angle_deg;
} balance_control_output_t;

typedef struct
{
    balance_control_config_t config;
    balance_control_output_t output;
} balance_control_t;

void balance_control_init(balance_control_t *control,
                          const balance_control_config_t *config);
void balance_control_reset(balance_control_t *control);
void balance_control_step(balance_control_t *control,
                          const balance_control_input_t *input);
const balance_control_output_t *balance_control_get_output(
    const balance_control_t *control);

#endif
