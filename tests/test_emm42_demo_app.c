#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control_config.h"
#include "emm42.h"
#include "emm42_demo_app.h"

static uint32 mock_now_ms;
static emm42_frame_t mock_frame;
static uint8 mock_frame_ready;
static float mock_position_deg;
static float mock_last_move_deg;
static uint32 mock_move_count;
static uint32 mock_query_count;

static void queue_position(float position_deg)
{
    mock_position_deg = position_deg;
    mock_frame.data[0] = EMM42_DEFAULT_ADDRESS;
    mock_frame.data[1] = 0x36u;
    mock_frame.data[7] = 0x6Bu;
    mock_frame.length = 8u;
    mock_frame_ready = 1u;
}

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void heartbeat_hw_uart_send_string(const char *message)
{
    (void)message;
}

void emm42_init(void)
{
}

uint8 emm42_set_current_position_zero(uint8 address)
{
    (void)address;
    return 1u;
}

uint8 emm42_set_enabled(uint8 address, uint8 enabled, uint8 synchronized)
{
    (void)address;
    (void)enabled;
    (void)synchronized;
    return 1u;
}

uint8 emm42_move_angle(uint8 address, float angle_deg, uint16 rpm,
                       uint8 acceleration, emm42_position_mode_enum mode,
                       uint8 synchronized)
{
    (void)address;
    (void)rpm;
    (void)acceleration;
    (void)mode;
    (void)synchronized;
    mock_last_move_deg = angle_deg;
    mock_move_count++;
    return 1u;
}

uint8 emm42_stop(uint8 address, uint8 synchronized)
{
    (void)address;
    (void)synchronized;
    return 1u;
}

uint8 emm42_query_position(uint8 address)
{
    (void)address;
    mock_query_count++;
    return 1u;
}

uint8 emm42_read_frame(emm42_frame_t *frame)
{
    if (0u == mock_frame_ready)
    {
        return 0u;
    }
    *frame = mock_frame;
    mock_frame_ready = 0u;
    return 1u;
}

uint8 emm42_decode_position_deg(const emm42_frame_t *frame, uint8 address,
                                float *position_deg)
{
    if ((8u != frame->length) || (address != frame->data[0]) ||
        (0x36u != frame->data[1]))
    {
        return 0u;
    }
    *position_deg = mock_position_deg;
    return 1u;
}

static void process_at(uint32 now_ms)
{
    mock_now_ms = now_ms;
    emm42_demo_app_process();
}

int main(void)
{
    float level_motor_deg;

    mock_now_ms = 0u;
    mock_frame_ready = 0u;
    mock_move_count = 0u;
    mock_query_count = 0u;
    emm42_demo_app_init();
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_POWER);
    assert(emm42_demo_app_get_target_angle_deg() ==
           BALANCE_STARTUP_LEVER_ANGLE_DEG);

    process_at(3000u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_ZERO);
    assert(mock_query_count == 0u);
    process_at(3100u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_ENABLE);
    assert(mock_query_count == 0u);
    process_at(3200u);
    process_at(3200u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_LEVEL);
    assert(mock_move_count == 1u);
    assert(mock_query_count == 0u);
    level_motor_deg = mock_last_move_deg;
    assert(fabsf(level_motor_deg - (-29.424f)) < 0.02f);
    assert(emm42_demo_app_get_target_angle_deg() == 0.0f);

    process_at(3300u);
    assert(mock_query_count > 0u);
    queue_position(level_motor_deg);
    process_at(3300u);
    assert(emm42_demo_app_is_motor_feedback_valid() != 0u);
    process_at(3500u);
    process_at(3500u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_POSITIVE);
    assert(fabsf(mock_last_move_deg - (-11.184f)) < 0.02f);
    assert(emm42_demo_app_get_target_angle_deg() == 5.0f);

    process_at(5000u);
    process_at(5000u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_NEGATIVE);
    assert(fabsf(mock_last_move_deg - (-52.379f)) < 0.02f);
    assert(emm42_demo_app_get_target_angle_deg() == -5.0f);

    puts("emm42 demo app tests passed");
    return 0;
}
