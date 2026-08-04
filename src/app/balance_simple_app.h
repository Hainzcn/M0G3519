#ifndef BALANCE_SIMPLE_APP_H_
#define BALANCE_SIMPLE_APP_H_

#include "zf_common_typedef.h"

typedef enum
{
    BALANCE_SIMPLE_DISABLED = 0,
    BALANCE_SIMPLE_STARTUP_LEVEL,
    BALANCE_SIMPLE_WAIT_VISION,
    BALANCE_SIMPLE_ACTIVE,
    BALANCE_SIMPLE_STATIC_LOCK,
    BALANCE_SIMPLE_SAFE_RETURN,
    BALANCE_SIMPLE_FAULT,
} balance_simple_state_enum;

typedef enum
{
    BALANCE_SIMPLE_FAULT_NONE = 0,
    BALANCE_SIMPLE_FAULT_NOT_CALIBRATED,
    BALANCE_SIMPLE_FAULT_LINKAGE,
    BALANCE_SIMPLE_FAULT_COMMAND,
    BALANCE_SIMPLE_FAULT_STARTUP_TIMEOUT,
    BALANCE_SIMPLE_FAULT_MOTOR_FEEDBACK,
    BALANCE_SIMPLE_FAULT_MOTOR_HARD_LIMIT,
    BALANCE_SIMPLE_FAULT_BALL_HARD_EDGE,
} balance_simple_fault_enum;

#define BALANCE_SIMPLE_FLAG_OBSERVER_VALID       (0x0001u)
#define BALANCE_SIMPLE_FLAG_MOTOR_POSITION_VALID (0x0002u)
#define BALANCE_SIMPLE_FLAG_COMMAND_PENDING      (0x0004u)
#define BALANCE_SIMPLE_FLAG_POSITION_ACTIVE      (0x0008u)
#define BALANCE_SIMPLE_FLAG_STATIC_LOCK_ENABLED  (0x0010u)
#define BALANCE_SIMPLE_FLAG_SOFT_BALL_EDGE        (0x0020u)
#define BALANCE_SIMPLE_FLAG_HARD_BALL_EDGE        (0x0040u)
#define BALANCE_SIMPLE_FLAG_MOTOR_VELOCITY_VALID  (0x0080u)
#define BALANCE_SIMPLE_FLAG_STOP_TEST_TUNING      (0x0100u)

typedef struct
{
    uint32 mcu_ms;
    uint16 vision_sequence;
    uint32 capture_ms;
    uint32 vision_age_ms;
    uint8 vision_confidence;
    uint8 vision_flags;
    float raw_position_m;
    float estimated_position_m;
    float estimated_velocity_mps;
    float target_position_m;
    float position_error_m;
    float velocity_limit_mps;
    float target_velocity_mps;
    float effective_kv_deg_per_mm;
    float target_beam_angle_deg;
    float measured_beam_angle_deg;
    float beam_angle_error_deg;
    float omega_command_deg_s;
    float motor_rpm_requested;
    int16 motor_rpm_command;
    int16 motor_rpm_actual;
    float motor_position_deg;
    uint32 motor_position_age_ms;
    uint32 motor_velocity_age_ms;
    float integral_velocity_mps;
    float filtered_ball_accel_mps2;
    float car_accel_mps2;
    float car_filtered_accel_mps2;
    float car_feedforward_angle_deg;
    float car_feedforward_scale;
    uint8 car_accel_valid;
    uint8 car_feedforward_active;
    balance_simple_state_enum state;
    balance_simple_fault_enum fault;
    uint16 flags;
    uint16 saturation_flags;
    uint16 command_error_count;
    uint16 emm42_rx_overflow_count;
} balance_simple_status_t;

void balance_simple_app_init(void);
void balance_simple_app_process(void);
uint8 balance_simple_app_start(void);
void balance_simple_app_set_stop_test_mode(uint8 enabled);
uint8 balance_simple_app_set_target_position_m(float target_position_m);
void balance_simple_app_set_vehicle_accel_mps2(float accel_mps2, uint8 valid);
void balance_simple_app_disable(void);
const balance_simple_status_t *balance_simple_app_get_status(void);

#endif
