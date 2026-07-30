#ifndef EMM42_H_
#define EMM42_H_

#include "zf_common_typedef.h"

#define EMM42_DEFAULT_ADDRESS          (1u)
#define EMM42_MAX_RPM                  (5000u)
#define EMM42_DEFAULT_PULSES_PER_REV   (3200u)
#define EMM42_FRAME_MAX_SIZE           (32u)

typedef enum
{
    EMM42_POSITION_RELATIVE_TO_TARGET = 0,
    EMM42_POSITION_ABSOLUTE = 1,
    EMM42_POSITION_RELATIVE_TO_CURRENT = 2,
} emm42_position_mode_enum;

typedef enum
{
    EMM42_HOME_NEAREST = 0,
    EMM42_HOME_DIRECTIONAL = 1,
    EMM42_HOME_STALL = 2,
    EMM42_HOME_LIMIT_SWITCH = 3,
} emm42_home_mode_enum;

typedef struct
{
    uint8 data[EMM42_FRAME_MAX_SIZE];
    uint8 length;
} emm42_frame_t;

void emm42_init(void);
uint8 emm42_set_enabled(uint8 address, uint8 enabled, uint8 synchronized);
uint8 emm42_run_velocity(uint8 address, int16 rpm, uint8 acceleration,
                         uint8 synchronized);
uint8 emm42_move_pulses(uint8 address, int32 pulses, uint16 rpm,
                        uint8 acceleration, emm42_position_mode_enum mode,
                        uint8 synchronized);
uint8 emm42_move_angle(uint8 address, float angle_deg, uint16 rpm,
                       uint8 acceleration, emm42_position_mode_enum mode,
                       uint8 synchronized);
uint8 emm42_stop(uint8 address, uint8 synchronized);
uint8 emm42_start_synchronized(uint8 address);
uint8 emm42_set_current_position_zero(uint8 address);
uint8 emm42_home(uint8 address, emm42_home_mode_enum mode, uint8 synchronized);
uint8 emm42_query_position(uint8 address);
uint8 emm42_query_velocity(uint8 address);

uint8 emm42_read_frame(emm42_frame_t *frame);
uint8 emm42_decode_ack(const emm42_frame_t *frame, uint8 address,
                       uint8 command, uint8 *status);
uint8 emm42_decode_position_deg(const emm42_frame_t *frame, uint8 address,
                                float *position_deg);
uint8 emm42_decode_velocity_rpm(const emm42_frame_t *frame, uint8 address,
                                int16 *velocity_rpm);
uint32 emm42_get_rx_overflow_count(void);

#endif
