#ifndef BALANCE_CONTROL_H_
#define BALANCE_CONTROL_H_

#include "zf_common_typedef.h"

#define BALANCE_CONTROL_FLAG_MEASUREMENT_FRESH  (0x0001u)
#define BALANCE_CONTROL_FLAG_PREDICT_ONLY       (0x0002u)
#define BALANCE_CONTROL_FLAG_EDGE_RECOVERY      (0x0004u)
#define BALANCE_CONTROL_FLAG_DYNAMICS_SATURATED (0x0008u)
#define BALANCE_CONTROL_FLAG_ANGLE_SATURATED    (0x0010u)
#define BALANCE_CONTROL_FLAG_SLEW_SATURATED     (0x0020u)
#define BALANCE_CONTROL_FLAG_HARD_EDGE          (0x0040u)
#define BALANCE_CONTROL_FLAG_VELOCITY_SATURATED (0x0080u)
#define BALANCE_CONTROL_FLAG_OVERSPEED_PULLBACK (0x0100u)
#define BALANCE_CONTROL_FLAG_PREDICTOR_DEGRADED (0x0200u)
#define BALANCE_CONTROL_FLAG_CAPTURE_ACTIVE     (0x0400u)
#define BALANCE_CONTROL_FLAG_BREAKAWAY_ACTIVE   (0x0800u)
#define BALANCE_CONTROL_FLAG_CALIBRATION_PENDING (0x1000u)

#define BALANCE_CONTROL_COMMAND_HISTORY_COUNT   (17u)

typedef enum
{
    BALANCE_CONTROL_PHASE_HOLD = 0,
    BALANCE_CONTROL_PHASE_ACCEL,
    BALANCE_CONTROL_PHASE_TRACK,
    BALANCE_CONTROL_PHASE_BRAKE,
    BALANCE_CONTROL_PHASE_OVERSPEED,
    BALANCE_CONTROL_PHASE_EDGE_RECOVERY,
    BALANCE_CONTROL_PHASE_CAPTURE,
} balance_control_phase_enum;

typedef enum
{
    BALANCE_FRICTION_MOTION = 0,
    BALANCE_FRICTION_BREAKAWAY,
    BALANCE_FRICTION_CAPTURE,
    BALANCE_FRICTION_STOPPED,
} balance_friction_mode_enum;

typedef struct
{
    float position_gain_s_inv;
    float velocity_gain_s_inv;
    float max_ball_velocity_mps;
    float rolling_factor;
    float rolling_friction_accel_mps2;
    float rail_curvature_m_inv;
    float position_correction_gain;
    float velocity_residual_gain;
    float max_ball_accel_mps2;
    float brake_accel_mps2;
    float actuator_delay_s;
    float brake_margin_delay_s;
    float overspeed_release_ratio;
    uint32 overspeed_min_hold_ms;
    float command_period_s;
    float capture_position_m;
    float center_dead_position_m;
    float capture_velocity_mps;
    float stick_velocity_mps;
    float capture_integral_gain;
    float capture_max_accel_mps2;
    float breakaway_angle_deg;
    uint32 breakaway_qualify_ms;
    uint32 breakaway_pulse_ms;
    float breakaway_movement_m;
    float max_lever_angle_deg;
    float degraded_lever_angle_deg;
    float edge_recovery_accel_mps2;
    float edge_position_m;
    float hard_edge_position_m;
    uint32 fresh_measurement_ms;
    uint32 valid_measurement_ms;
    uint8 calibration_provisional;
} balance_control_config_t;

typedef struct
{
    uint8 new_measurement;
    uint8 measurement_valid;
    float measured_position_m;
    float measured_velocity_mps;
    float measurement_interval_s;
    uint32 measurement_age_ms;
    float target_position_m;
    float reference_position_m;
    float reference_velocity_mps;
    float feedforward_accel_mps2;
    uint8 reference_holding;
    float car_accel_mps2;
    uint8 actual_lever_valid;
    float actual_lever_angle_deg;
    uint8 actuator_command_updated;
    float actuator_command_angle_deg;
    uint8 update_control_output;
    float dt_s;
} balance_control_input_t;

typedef struct
{
    uint8 has_state;
    uint16 flags;
    float estimated_position_m;
    float estimated_velocity_mps;
    float predicted_position_m;
    float predicted_velocity_mps;
    float position_error_m;
    float velocity_command_mps;
    float velocity_limit_mps;
    float brake_distance_m;
    float feedforward_accel_mps2;
    float feedback_accel_mps2;
    float desired_ball_accel_mps2;
    float lever_angle_deg;
    balance_control_phase_enum phase;
    balance_friction_mode_enum friction_mode;
} balance_control_output_t;

typedef struct
{
    balance_control_config_t config;
    balance_control_output_t output;
    float command_history[BALANCE_CONTROL_COMMAND_HISTORY_COUNT];
    uint8 command_history_head;
    uint8 command_history_count;
    uint32 stuck_elapsed_ms;
    uint32 breakaway_remaining_ms;
    float stuck_anchor_position_m;
    uint8 stuck_anchor_valid;
    uint32 overspeed_hold_remaining_ms;
    float overspeed_accel_sign;
    float overspeed_target_position_m;
    uint8 overspeed_active;
    float capture_integral;
} balance_control_t;

void balance_control_init(balance_control_t *control,
                          const balance_control_config_t *config);
void balance_control_reset(balance_control_t *control);
void balance_control_step(balance_control_t *control,
                          const balance_control_input_t *input);
const balance_control_output_t *balance_control_get_output(
    const balance_control_t *control);

#endif
