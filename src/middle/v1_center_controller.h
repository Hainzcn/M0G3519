#ifndef V1_CENTER_CONTROLLER_H_
#define V1_CENTER_CONTROLLER_H_

#include "zf_common_typedef.h"

typedef enum
{
    V1_CENTER_WAIT_VISION = 0,
    V1_CENTER_HOLD,
    V1_CENTER_ACCEL,
    V1_CENTER_CRUISE,
    V1_CENTER_BRAKE,
    V1_CENTER_SETTLE,
} v1_center_phase_enum;

typedef enum
{
    V1_CENTER_FAULT_NONE = 0,
    V1_CENTER_FAULT_CAPTURE_TIMEOUT,
} v1_center_fault_enum;

typedef struct
{
    float drive_angle_offset_deg;
    float brake_angle_offset_deg;
    float brake_decel_mps2;
} v1_center_direction_config_t;

typedef struct
{
    v1_center_direction_config_t positive;
    v1_center_direction_config_t negative;
    float neutral_angle_deg;
    float max_velocity_mps;
    float resume_velocity_mps;
    float brake_delay_s;
    float brake_margin_m;
    float hold_enter_position_m;
    float hold_exit_position_m;
    float stop_velocity_mps;
    float hold_exit_velocity_mps;
    uint32 settle_ms;
    uint32 capture_timeout_ms;
    uint32 max_measurement_age_ms;
    uint8 recovery_valid_frames;
} v1_center_config_t;

typedef struct
{
    uint8 valid;
    uint8 new_measurement;
    float position_m;
    float velocity_mps;
    uint32 age_ms;
} v1_center_observation_t;

typedef struct
{
    v1_center_phase_enum phase;
    v1_center_fault_enum fault;
    int8 direction;
    float target_angle_deg;
    float remaining_m;
    float brake_distance_m;
} v1_center_output_t;

typedef struct
{
    const v1_center_config_t *config;
    v1_center_output_t output;
    uint32 phase_start_ms;
    uint32 capture_start_ms;
    uint32 settle_valid_start_ms;
    uint8 capture_active;
    uint8 settle_valid;
    uint8 recovery_valid_count;
} v1_center_controller_t;

void v1_center_controller_init(v1_center_controller_t *self,
                               const v1_center_config_t *config);
void v1_center_controller_reset(v1_center_controller_t *self);
void v1_center_controller_begin(v1_center_controller_t *self,
                                float position_m, float velocity_mps,
                                uint32 now_ms);
void v1_center_controller_step(v1_center_controller_t *self,
                               const v1_center_observation_t *observation,
                               uint32 now_ms);
const v1_center_output_t *v1_center_controller_get_output(
    const v1_center_controller_t *self);

#endif
