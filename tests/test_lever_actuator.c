#include <assert.h>
#include <stdio.h>

#include "balance_linkage.h"
#include "emm42.h"
#include "lever_actuator.h"

static uint32 disable_count;
static uint32 zero_count;
static uint8 frame_ready;
static uint8 split_frame_stage;
static emm42_frame_t queued_frame;

uint8 balance_linkage_relative_motor_deg(float reference_lever_angle_deg,
                                         float target_lever_angle_deg,
                                         float *relative_motor_deg)
{
    (void)reference_lever_angle_deg;
    *relative_motor_deg = target_lever_angle_deg;
    return 1u;
}

void emm42_init(void) {}
uint32 emm42_get_rx_overflow_count(void) { return 0u; }
uint8 emm42_stop(uint8 address, uint8 synchronized)
{
    (void)address; (void)synchronized; return 1u;
}
uint8 emm42_set_enabled(uint8 address, uint8 enabled, uint8 synchronized)
{
    (void)address; (void)synchronized;
    if (0u == enabled) { disable_count++; }
    return 1u;
}
uint8 emm42_set_current_position_zero(uint8 address)
{
    (void)address; zero_count++; return 1u;
}
uint8 emm42_move_angle(uint8 address, float angle_deg, uint16 rpm,
                       uint8 acceleration, emm42_position_mode_enum mode,
                       uint8 synchronized)
{
    (void)address; (void)angle_deg; (void)rpm; (void)acceleration;
    (void)mode; (void)synchronized; return 1u;
}
uint8 emm42_query_position(uint8 address) { (void)address; return 1u; }
uint8 emm42_read_frame(emm42_frame_t *frame)
{
    if (1u == split_frame_stage)
    {
        assert(0u == frame->length);
        frame->data[0] = EMM42_DEFAULT_ADDRESS;
        frame->data[1] = 0xF3u;
        frame->length = 2u;
        split_frame_stage = 2u;
        return 0u;
    }
    if (2u == split_frame_stage)
    {
        assert(2u == frame->length);
        frame->data[2] = 0x02u;
        frame->data[3] = 0x6Bu;
        frame->length = 4u;
        split_frame_stage = 0u;
        return 1u;
    }
    if (0u == frame_ready) { return 0u; }
    *frame = queued_frame; frame_ready = 0u; return 1u;
}
uint8 emm42_decode_ack(const emm42_frame_t *frame, uint8 address,
                       uint8 command, uint8 *status)
{
    if ((frame->data[0] != address) || (frame->data[1] != command))
    {
        return 0u;
    }
    *status = frame->data[2];
    return 1u;
}
uint8 emm42_decode_position_deg(const emm42_frame_t *frame, uint8 address,
                                float *position_deg)
{
    (void)frame; (void)address; (void)position_deg; return 0u;
}

static void queue_ack(uint8 command)
{
    queued_frame.data[0] = EMM42_DEFAULT_ADDRESS;
    queued_frame.data[1] = command;
    queued_frame.data[2] = 0x02u;
    queued_frame.length = 4u;
    frame_ready = 1u;
}

int main(void)
{
    const lever_actuator_config_t config =
    {
        1u, -5.0f, 0.0f, 4.0f, -1, -1, 30u, 20u,
        0u, 0u, 10u, 2500u, 200u, 100u, 1000u,
        0.1f, 1.0f, 5.0f, 3u,
    };
    lever_actuator_t actuator;

    lever_actuator_init(&actuator, &config, 0u);
    lever_actuator_process(&actuator, 0u);
    assert(LEVER_ACTUATOR_DISABLING == actuator.status.state);
    assert(1u == disable_count);

    split_frame_stage = 1u;
    lever_actuator_process(&actuator, 1u);
    assert(LEVER_ACTUATOR_DISABLING == actuator.status.state);
    lever_actuator_process(&actuator, 2u);
    assert(LEVER_ACTUATOR_ZEROING == actuator.status.state);
    assert(1u == zero_count);

    /* Exercise timeout retry independently after a fresh startup. */
    disable_count = 0u;
    zero_count = 0u;
    lever_actuator_init(&actuator, &config, 0u);
    lever_actuator_process(&actuator, 0u);
    lever_actuator_process(&actuator, 11u);
    assert(LEVER_ACTUATOR_DISABLING == actuator.status.state);
    assert(2u == disable_count);
    assert(1u == actuator.status.command_error_count);

    queue_ack(0xF3u);
    lever_actuator_process(&actuator, 12u);
    assert(LEVER_ACTUATOR_ZEROING == actuator.status.state);
    assert(1u == zero_count);

    lever_actuator_process(&actuator, 23u);
    assert(LEVER_ACTUATOR_ZEROING == actuator.status.state);
    assert(2u == zero_count);
    assert(2u == actuator.status.command_error_count);

    puts("lever actuator retry tests passed");
    return 0;
}
