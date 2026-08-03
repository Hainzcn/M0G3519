#ifndef BALANCE_APP_H_
#define BALANCE_APP_H_

#include "zf_common_typedef.h"
#include "balance_control.h"
#include "ball_motion_profile.h"

typedef enum
{
    BALANCE_APP_UNCONFIGURED = 0,
    BALANCE_APP_POWER_WAIT,
    BALANCE_APP_SET_REFERENCE,
    BALANCE_APP_ENABLE,
    BALANCE_APP_MOVE_LEVEL,
    BALANCE_APP_ACTIVE,
    BALANCE_APP_RECOVERY,
    BALANCE_APP_FAULT,
    BALANCE_APP_DISABLE,
    BALANCE_APP_WAIT_LOWER_STOP,
    BALANCE_APP_WAIT_VISION,
} balance_app_state_enum;

typedef enum
{
    BALANCE_FAULT_NONE = 0,
    BALANCE_FAULT_LINKAGE_UNREACHABLE,
    BALANCE_FAULT_COMMAND_TIMEOUT,
    BALANCE_FAULT_COMMAND_REJECTED,
    BALANCE_FAULT_MOVE_LEVEL_TIMEOUT,
    BALANCE_FAULT_MOTOR_FOLLOW_ERROR,
    BALANCE_FAULT_BALL_HARD_EDGE,
} balance_app_fault_enum;

typedef enum
{
    BALANCE_SEQUENCE_IDLE = 0,
    BALANCE_SEQUENCE_TO_POSITIVE,
    BALANCE_SEQUENCE_TO_NEGATIVE,
    BALANCE_SEQUENCE_COMPLETE,
    BALANCE_SEQUENCE_CANCELED,
    BALANCE_SEQUENCE_TIMEOUT,
} balance_app_sequence_state_enum;

#define BALANCE_APP_FLAG_ACTIVE                 (0x01u)
#define BALANCE_APP_FLAG_MOTOR_FEEDBACK_VALID   (0x02u)
#define BALANCE_APP_FLAG_LINK_ONLINE            (0x04u)
#define BALANCE_APP_FLAG_MEASUREMENT_ACCEPTED   (0x08u)
#define BALANCE_APP_FLAG_COMMAND_PENDING        (0x10u)
#define BALANCE_APP_FLAG_FAULT_LATCHED          (0x20u)
#define BALANCE_APP_FLAG_LEVER_FEEDBACK_VALID   (0x40u)
#define BALANCE_APP_FLAG_SEQUENCE_ACTIVE        (0x80u)

typedef struct
{
    balance_app_state_enum state;
    balance_app_fault_enum fault;
    uint8 flags;
    uint16 control_flags;
    uint8 vision_confidence;
    uint8 vision_raw_flags;
    uint8 vision_raw_confidence;
    uint16 vision_sequence;
    uint16 vision_raw_sequence;
    uint32 vision_age_ms;
    int16 vision_raw_position_dmm;
    int16 vision_raw_velocity_mm_s;
    float estimated_position_m;
    float estimated_velocity_mps;
    float predicted_position_m;
    float predicted_velocity_mps;
    float target_position_m;
    float reference_position_m;
    float reference_velocity_mps;
    float reference_accel_mps2;
    float position_error_m;
    float velocity_command_mps;
    float velocity_limit_mps;
    float brake_distance_m;
    float feedforward_accel_mps2;
    float feedback_accel_mps2;
    float desired_ball_accel_mps2;
    float car_encoder_speed_mps;
    float car_encoder_accel_mps2;
    float car_imu_accel_mps2;
    float car_feedforward_accel_mps2;
    float car_sync_lever_angle_deg;
    uint32 car_imu_accel_age_ms;
    uint8 car_imu_accel_valid;
    float raw_lever_angle_deg;
    float lever_angle_deg;
    float actual_lever_angle_deg;
    float motor_target_deg;
    float motor_feedback_deg;
    uint16 command_error_count;
    uint16 emm42_rx_overflow_count;
    uint32 sequence_elapsed_ms;
    ball_motion_phase_enum motion_phase;
    balance_control_phase_enum control_phase;
    balance_friction_mode_enum friction_mode;
    balance_app_sequence_state_enum sequence_state;
} balance_app_status_t;

void balance_app_init(void);
void balance_app_process(void);
uint8 balance_app_set_target_position_m(float target_position_m);
uint8 balance_app_start_sequence(void);
void balance_app_cancel_motion(void);
const balance_app_status_t *balance_app_get_status(void);

#endif
