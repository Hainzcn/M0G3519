#ifndef SW1_OPEN_LOOP_H_
#define SW1_OPEN_LOOP_H_

#include "zf_common_typedef.h"

#define SW1_OPEN_LOOP_PULSE_COUNT              (4u)

typedef enum
{
    SW1_OPEN_LOOP_IDLE = 0,
    SW1_OPEN_LOOP_POS_ACCEL,
    SW1_OPEN_LOOP_POS_BRAKE,
    SW1_OPEN_LOOP_TURN_DWELL,
    SW1_OPEN_LOOP_NEG_ACCEL,
    SW1_OPEN_LOOP_NEG_BRAKE,
    SW1_OPEN_LOOP_FINAL_SETTLE,
    SW1_OPEN_LOOP_COMPLETE,
    SW1_OPEN_LOOP_CANCELED,
    SW1_OPEN_LOOP_FAULT,
} sw1_open_loop_phase_enum;

typedef enum
{
    SW1_OPEN_LOOP_FAULT_NONE = 0,
    SW1_OPEN_LOOP_FAULT_DEADLINE_MISSED,
    SW1_OPEN_LOOP_FAULT_TOTAL_TIMEOUT,
} sw1_open_loop_fault_enum;

typedef struct
{
    float angle_offset_deg;
    uint32 duration_ms;
} sw1_open_loop_pulse_t;

typedef struct
{
    sw1_open_loop_pulse_t pulse[SW1_OPEN_LOOP_PULSE_COUNT];
    float neutral_angle_deg;
    uint32 turn_dwell_ms;
    uint32 final_settle_ms;
    uint32 max_deadline_late_ms;
    uint32 total_timeout_ms;
} sw1_open_loop_config_t;

typedef struct
{
    sw1_open_loop_phase_enum phase;
    sw1_open_loop_fault_enum fault;
    float target_angle_deg;
    uint32 elapsed_ms;
    uint8 active;
    uint8 command_due;
} sw1_open_loop_output_t;

typedef struct
{
    const sw1_open_loop_config_t *config;
    sw1_open_loop_output_t output;
    uint32 start_ms;
    uint32 command_deadline_ms;
} sw1_open_loop_t;

void sw1_open_loop_init(sw1_open_loop_t *self,
                        const sw1_open_loop_config_t *config);
float sw1_open_loop_get_start_angle(const sw1_open_loop_t *self);
void sw1_open_loop_start(sw1_open_loop_t *self, uint32 now_ms);
void sw1_open_loop_step(sw1_open_loop_t *self, uint32 now_ms);
void sw1_open_loop_mark_command_applied(sw1_open_loop_t *self);
void sw1_open_loop_cancel(sw1_open_loop_t *self);
const sw1_open_loop_output_t *sw1_open_loop_get_output(
    const sw1_open_loop_t *self);

#endif
