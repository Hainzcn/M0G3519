#ifndef LEVER_ACTUATOR_H_
#define LEVER_ACTUATOR_H_

#include "zf_common_typedef.h"
#include "emm42.h"

typedef enum
{
    LEVER_ACTUATOR_UNCONFIGURED = 0,
    LEVER_ACTUATOR_POWER_WAIT,
    LEVER_ACTUATOR_DISABLING,
    LEVER_ACTUATOR_LOWER_SETTLE,
    LEVER_ACTUATOR_ZEROING,
    LEVER_ACTUATOR_ENABLING,
    LEVER_ACTUATOR_LEVELING,
    LEVER_ACTUATOR_READY,
    LEVER_ACTUATOR_FAULT,
} lever_actuator_state_enum;

typedef enum
{
    LEVER_ACTUATOR_FAULT_NONE = 0,
    LEVER_ACTUATOR_FAULT_LINKAGE,
    LEVER_ACTUATOR_FAULT_COMMAND_TIMEOUT,
    LEVER_ACTUATOR_FAULT_COMMAND_REJECTED,
    LEVER_ACTUATOR_FAULT_LEVEL_TIMEOUT,
    LEVER_ACTUATOR_FAULT_FOLLOW_ERROR,
} lever_actuator_fault_enum;

typedef enum
{
    LEVER_COMMAND_STARTED = 0,
    LEVER_COMMAND_UNCHANGED,
    LEVER_COMMAND_BUSY,
    LEVER_COMMAND_NOT_READY,
    LEVER_COMMAND_OUT_OF_RANGE,
    LEVER_COMMAND_FAULT,
} lever_command_result_enum;

typedef struct
{
    uint8 startup_calibrated;
    float startup_reference_angle_deg;
    float neutral_angle_deg;
    float max_abs_angle_deg;
    int8 linkage_target_sign;
    int8 motor_direction_sign;
    uint16 move_rpm;
    uint8 acceleration;
    uint32 power_wait_ms;
    uint32 lower_settle_ms;
    uint32 command_timeout_ms;
    uint32 level_timeout_ms;
    uint32 level_settle_ms;
    uint32 query_period_ms;
    uint32 follow_error_timeout_ms;
    float command_deadband_deg;
    float level_motor_tolerance_deg;
    float follow_error_deg;
    uint8 max_consecutive_errors;
} lever_actuator_config_t;

typedef struct
{
    lever_actuator_state_enum state;
    lever_actuator_fault_enum fault;
    uint8 command_pending;
    uint8 motor_feedback_valid;
    float target_angle_deg;
    float motor_target_deg;
    float motor_feedback_deg;
    uint16 command_error_count;
    uint16 rx_overflow_count;
} lever_actuator_status_t;

typedef struct
{
    const lever_actuator_config_t *config;
    lever_actuator_status_t status;
    uint32 state_start_ms;
    uint32 pending_since_ms;
    uint32 last_query_ms;
    uint32 level_tolerance_start_ms;
    uint32 follow_error_start_ms;
    emm42_frame_t response_frame;
    uint8 pending_command;
    uint8 consecutive_errors;
    uint8 level_move_acked;
    uint8 level_tolerance_active;
    uint8 follow_error_active;
} lever_actuator_t;

void lever_actuator_init(lever_actuator_t *self,
                         const lever_actuator_config_t *config,
                         uint32 now_ms);
void lever_actuator_process(lever_actuator_t *self, uint32 now_ms);
lever_command_result_enum lever_actuator_command_angle(
    lever_actuator_t *self, float angle_deg, uint32 now_ms);
lever_command_result_enum lever_actuator_command_neutral(
    lever_actuator_t *self, uint32 now_ms);
uint8 lever_actuator_is_ready(const lever_actuator_t *self);
const lever_actuator_status_t *lever_actuator_get_status(
    const lever_actuator_t *self);

#endif
