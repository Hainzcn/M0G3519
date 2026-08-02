#ifndef BALANCE_APP_H_
#define BALANCE_APP_H_

#include "zf_common_typedef.h"

typedef enum
{
    BALANCE_MODE_STARTUP = 0,
    BALANCE_MODE_V1,
    BALANCE_MODE_SW1,
    BALANCE_MODE_EDGE_RECOVERY,
    BALANCE_MODE_COMPLETE,
    BALANCE_MODE_FAULT,
} balance_mode_enum;

typedef enum
{
    BALANCE_FAULT_NONE = 0,
    BALANCE_FAULT_LINKAGE_UNREACHABLE,
    BALANCE_FAULT_COMMAND_TIMEOUT,
    BALANCE_FAULT_COMMAND_REJECTED,
    BALANCE_FAULT_LEVEL_TIMEOUT,
    BALANCE_FAULT_MOTOR_FOLLOW_ERROR,
    BALANCE_FAULT_EDGE_NO_PROGRESS,
    BALANCE_FAULT_SW1_DEADLINE_MISSED,
    BALANCE_FAULT_SW1_TIMEOUT,
    BALANCE_FAULT_V1_CAPTURE_TIMEOUT,
} balance_app_fault_enum;

typedef enum
{
    BALANCE_REQUEST_ACCEPTED = 0,
    BALANCE_REQUEST_NOT_READY,
    BALANCE_REQUEST_BUSY,
    BALANCE_REQUEST_FAULT,
} balance_request_result_t;

#define BALANCE_APP_FLAG_ACTUATOR_READY       (0x01u)
#define BALANCE_APP_FLAG_MOTOR_FEEDBACK_VALID (0x02u)
#define BALANCE_APP_FLAG_VISION_ONLINE        (0x04u)
#define BALANCE_APP_FLAG_MEASUREMENT_FRESH    (0x08u)
#define BALANCE_APP_FLAG_COMMAND_PENDING      (0x10u)
#define BALANCE_APP_FLAG_FAULT_LATCHED        (0x20u)
#define BALANCE_APP_FLAG_SOFT_EDGE            (0x40u)
#define BALANCE_APP_FLAG_SW1_ACTIVE           (0x80u)

typedef struct
{
    uint8 valid;
    float longitudinal_accel_mps2;
    uint32 sample_ms;
} balance_platform_motion_t;

typedef struct
{
    balance_mode_enum mode;
    uint8 phase;
    balance_app_fault_enum fault;
    uint8 flags;
    uint16 vision_sequence;
    uint32 vision_age_ms;
    uint8 vision_confidence;
    float position_m;
    float velocity_mps;
    float remaining_m;
    float brake_distance_m;
    float lever_target_deg;
    float motor_target_deg;
    float motor_feedback_deg;
    uint32 sw1_elapsed_ms;
    uint16 command_error_count;
    uint16 emm42_rx_overflow_count;
} balance_app_status_t;

void balance_app_init(void);
void balance_app_process(void);
balance_request_result_t balance_app_start_sw1(void);
void balance_app_cancel(void);
const balance_app_status_t *balance_app_get_status(void);
void balance_app_set_platform_motion(const balance_platform_motion_t *motion);

#endif
