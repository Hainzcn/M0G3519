#include "emm42.h"

#include "emm42_hw.h"

#define EMM42_FRAME_END               (0x6Bu)

static uint8 emm42_send(const uint8 *frame, uint8 length)
{
    emm42_hw_clear_rx();
    return emm42_hw_write(frame, length);
}

static uint16 emm42_clamp_rpm(uint16 rpm)
{
    return (rpm > EMM42_MAX_RPM) ? EMM42_MAX_RPM : rpm;
}

static uint32 emm42_abs_pulses(int32 pulses)
{
    if (pulses >= 0)
    {
        return (uint32)pulses;
    }
    return (uint32)(-(pulses + 1)) + 1u;
}

static uint8 emm42_response_length(uint8 command)
{
    if (0x35u == command)
    {
        return 6u;
    }
    if (0x36u == command)
    {
        return 8u;
    }
    return 4u;
}

void emm42_init(void)
{
    emm42_hw_init();
}

uint8 emm42_set_enabled(uint8 address, uint8 enabled, uint8 synchronized)
{
    uint8 frame[6] = {address, 0xF3u, 0xABu,
                      (enabled != 0u) ? 1u : 0u,
                      (synchronized != 0u) ? 1u : 0u, EMM42_FRAME_END};
    return emm42_send(frame, (uint8)sizeof(frame));
}

uint8 emm42_run_velocity(uint8 address, int16 rpm, uint8 acceleration,
                         uint8 synchronized)
{
    uint16 speed;
    uint8 direction = 0u;
    uint8 frame[8];

    if (rpm < 0)
    {
        direction = 1u;
        speed = (uint16)(-(rpm + 1)) + 1u;
    }
    else
    {
        speed = (uint16)rpm;
    }
    speed = emm42_clamp_rpm(speed);

    frame[0] = address;
    frame[1] = 0xF6u;
    frame[2] = direction;
    frame[3] = (uint8)(speed >> 8);
    frame[4] = (uint8)speed;
    frame[5] = acceleration;
    frame[6] = (synchronized != 0u) ? 1u : 0u;
    frame[7] = EMM42_FRAME_END;
    return emm42_send(frame, (uint8)sizeof(frame));
}

uint8 emm42_move_pulses(uint8 address, int32 pulses, uint16 rpm,
                        uint8 acceleration, emm42_position_mode_enum mode,
                        uint8 synchronized)
{
    uint32 pulse_count = emm42_abs_pulses(pulses);
    uint8 frame[13];

    if (mode > EMM42_POSITION_RELATIVE_TO_CURRENT)
    {
        return 0u;
    }
    rpm = emm42_clamp_rpm(rpm);

    frame[0] = address;
    frame[1] = 0xFDu;
    frame[2] = (pulses < 0) ? 1u : 0u;
    frame[3] = (uint8)(rpm >> 8);
    frame[4] = (uint8)rpm;
    frame[5] = acceleration;
    frame[6] = (uint8)(pulse_count >> 24);
    frame[7] = (uint8)(pulse_count >> 16);
    frame[8] = (uint8)(pulse_count >> 8);
    frame[9] = (uint8)pulse_count;
    frame[10] = (uint8)mode;
    frame[11] = (synchronized != 0u) ? 1u : 0u;
    frame[12] = EMM42_FRAME_END;
    return emm42_send(frame, (uint8)sizeof(frame));
}

uint8 emm42_move_angle(uint8 address, float angle_deg, uint16 rpm,
                       uint8 acceleration, emm42_position_mode_enum mode,
                       uint8 synchronized)
{
    float pulses_float = angle_deg * (float)EMM42_DEFAULT_PULSES_PER_REV / 360.0f;
    int32 pulses = (int32)(pulses_float + ((pulses_float >= 0.0f) ? 0.5f : -0.5f));

    return emm42_move_pulses(address, pulses, rpm, acceleration, mode,
                             synchronized);
}

uint8 emm42_stop(uint8 address, uint8 synchronized)
{
    uint8 frame[5] = {address, 0xFEu, 0x98u,
                      (synchronized != 0u) ? 1u : 0u, EMM42_FRAME_END};
    return emm42_send(frame, (uint8)sizeof(frame));
}

uint8 emm42_start_synchronized(uint8 address)
{
    uint8 frame[4] = {address, 0xFFu, 0x66u, EMM42_FRAME_END};
    return emm42_send(frame, (uint8)sizeof(frame));
}

uint8 emm42_set_current_position_zero(uint8 address)
{
    uint8 frame[4] = {address, 0x0Au, 0x6Du, EMM42_FRAME_END};
    return emm42_send(frame, (uint8)sizeof(frame));
}

uint8 emm42_home(uint8 address, emm42_home_mode_enum mode, uint8 synchronized)
{
    uint8 frame[5];

    if (mode > EMM42_HOME_LIMIT_SWITCH)
    {
        return 0u;
    }
    frame[0] = address;
    frame[1] = 0x9Au;
    frame[2] = (uint8)mode;
    frame[3] = (synchronized != 0u) ? 1u : 0u;
    frame[4] = EMM42_FRAME_END;
    return emm42_send(frame, (uint8)sizeof(frame));
}

uint8 emm42_query_position(uint8 address)
{
    uint8 frame[3] = {address, 0x36u, EMM42_FRAME_END};
    return emm42_send(frame, (uint8)sizeof(frame));
}

uint8 emm42_query_velocity(uint8 address)
{
    uint8 frame[3] = {address, 0x35u, EMM42_FRAME_END};
    return emm42_send(frame, (uint8)sizeof(frame));
}

uint8 emm42_read_frame(emm42_frame_t *frame)
{
    uint8 byte;
    uint8 expected_length;

    if (NULL == frame)
    {
        return 0u;
    }

    /* A completed frame can be reused directly for the next response. */
    if ((frame->length > 0u) &&
        (frame->length <= EMM42_FRAME_MAX_SIZE) &&
        (EMM42_FRAME_END == frame->data[frame->length - 1u]))
    {
        frame->length = 0u;
    }

    while (0u != emm42_hw_read_byte(&byte))
    {
        if (frame->length >= EMM42_FRAME_MAX_SIZE)
        {
            frame->length = 0u;
        }
        frame->data[frame->length] = byte;
        frame->length ++;
        if (frame->length >= 2u)
        {
            expected_length = emm42_response_length(frame->data[1]);
            if (frame->length == expected_length)
            {
                if (EMM42_FRAME_END == byte)
                {
                    return 1u;
                }
                frame->length = 0u;
            }
        }
    }
    return 0u;
}

uint8 emm42_decode_ack(const emm42_frame_t *frame, uint8 address,
                       uint8 command, uint8 *status)
{
    if ((NULL == frame) || (NULL == status) || (4u != frame->length) ||
        (address != frame->data[0]) || (command != frame->data[1]) ||
        (EMM42_FRAME_END != frame->data[3]))
    {
        return 0u;
    }
    *status = frame->data[2];
    return 1u;
}

uint8 emm42_decode_position_deg(const emm42_frame_t *frame, uint8 address,
                                float *position_deg)
{
    uint32 raw;

    if ((NULL == frame) || (NULL == position_deg) || (8u != frame->length) ||
        (address != frame->data[0]) || (0x36u != frame->data[1]) ||
        (EMM42_FRAME_END != frame->data[7]))
    {
        return 0u;
    }
    raw = ((uint32)frame->data[3] << 24) |
          ((uint32)frame->data[4] << 16) |
          ((uint32)frame->data[5] << 8) |
          (uint32)frame->data[6];
    *position_deg = (float)raw * 360.0f / 65536.0f;
    if (0u != frame->data[2])
    {
        *position_deg = -*position_deg;
    }
    return 1u;
}

uint8 emm42_decode_velocity_rpm(const emm42_frame_t *frame, uint8 address,
                                int16 *velocity_rpm)
{
    uint16 raw;

    if ((NULL == frame) || (NULL == velocity_rpm) || (6u != frame->length) ||
        (address != frame->data[0]) || (0x35u != frame->data[1]) ||
        (EMM42_FRAME_END != frame->data[5]))
    {
        return 0u;
    }
    raw = ((uint16)frame->data[3] << 8) | (uint16)frame->data[4];
    *velocity_rpm = (0u != frame->data[2]) ? -(int16)raw : (int16)raw;
    return 1u;
}

uint32 emm42_get_rx_overflow_count(void)
{
    return emm42_hw_get_rx_overflow_count();
}
