#ifndef BALL_VELOCITY_CONTROLLER_H_
#define BALL_VELOCITY_CONTROLLER_H_

#include "zf_common_typedef.h"

#define BALL_VELOCITY_CONTROL_POSITION_ACTIVE (0x0001u)
#define BALL_VELOCITY_CONTROL_VELOCITY_LIMITED (0x0002u)
#define BALL_VELOCITY_CONTROL_OMEGA_LIMITED    (0x0004u)
#define BALL_VELOCITY_CONTROL_NEAR_DAMPING     (0x0008u)
#define BALL_VELOCITY_CONTROL_INTEGRAL_ACTIVE  (0x0010u)
#define BALL_VELOCITY_CONTROL_ANGLE_LIMITED     (0x0020u)
#define BALL_VELOCITY_CONTROL_ANGLE_SLEWED      (0x0040u)

typedef struct
{
    float position_kp_s_inv;
    float position_ki_s2_inv;
    float velocity_kv_deg_per_mmps;
    float acceleration_ka_deg_per_mps2;
    float position_on_m;
    float position_off_m;
    float max_target_velocity_mps;
    float braking_envelope_mps2;
    float actuator_delay_s;
    float max_target_beam_angle_deg;
    float target_beam_angle_slew_deg_s;
    float beam_angle_kp_s_inv;
    float beam_angle_deadband_deg;
    float max_beam_velocity_deg_s;
    float integral_zone_m;
    float integral_velocity_limit_mps;
    float near_position_m;
    float near_gain;
    float near_scale_max;
    float acceleration_filter_alpha;
} ball_velocity_controller_config_t;

typedef struct
{
    float position_m;
    float velocity_mps;
    float target_position_m;
    float measurement_dt_s;
    float control_dt_s;
    float measured_beam_angle_deg;
    uint8 new_measurement;
    uint8 observer_valid;
    uint8 output_saturated;
    uint8 freeze_integral;
} ball_velocity_controller_input_t;

typedef struct
{
    float position_error_m;
    float velocity_limit_mps;
    float target_velocity_mps;
    float velocity_error_mps;
    float effective_kv_deg_per_mm;
    float target_beam_angle_deg;
    float beam_angle_error_deg;
    float beam_velocity_deg_s;
    float integral_velocity_mps;
    float filtered_acceleration_mps2;
    uint16 flags;
} ball_velocity_controller_output_t;

typedef struct
{
    ball_velocity_controller_config_t config;
    ball_velocity_controller_output_t output;
    float previous_velocity_mps;
    float target_beam_angle_deg;
    uint8 position_active;
    uint8 has_previous_velocity;
    uint8 target_beam_angle_initialized;
} ball_velocity_controller_t;

void ball_velocity_controller_init(
    ball_velocity_controller_t *controller,
    const ball_velocity_controller_config_t *config);
void ball_velocity_controller_reset(ball_velocity_controller_t *controller);
void ball_velocity_controller_step(
    ball_velocity_controller_t *controller,
    const ball_velocity_controller_input_t *input);
const ball_velocity_controller_output_t *ball_velocity_controller_get_output(
    const ball_velocity_controller_t *controller);

#endif
