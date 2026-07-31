#include <assert.h>
#include <stdio.h>

#include "balance_app.h"
#include "balance_linkage.h"
#include "control_config.h"
#include "emm42.h"
#include "vision_link.h"

static uint32 mock_now_ms;
static emm42_frame_t mock_frame;
static uint8 mock_frame_ready;
static float mock_position_deg;
static uint32 mock_send_count;
static uint32 mock_move_count;
static uint32 mock_position_query_count;

#if (BALANCE_STARTUP_CALIBRATED != 0u)
static float level_motor_position(void)
{
    float position_deg;

    assert(0u != balance_linkage_relative_motor_deg(
        BALANCE_STARTUP_LEVER_ANGLE_DEG, 0.0f, &position_deg));
    return position_deg;
}

static void queue_ack(uint8 command)
{
    mock_frame.data[0] = EMM42_DEFAULT_ADDRESS;
    mock_frame.data[1] = command;
    mock_frame.data[2] = 0x02u;
    mock_frame.data[3] = 0x6Bu;
    mock_frame.length = 4u;
    mock_frame_ready = 1u;
}
#endif

#if (BALANCE_STARTUP_CALIBRATED != 0u)
static void queue_position(float position_deg)
{
    mock_position_deg = position_deg;
    mock_frame.data[0] = EMM42_DEFAULT_ADDRESS;
    mock_frame.data[1] = 0x36u;
    mock_frame.data[7] = 0x6Bu;
    mock_frame.length = 8u;
    mock_frame_ready = 1u;
}
#endif

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

uint8 emm42_set_enabled(uint8 address, uint8 enabled, uint8 synchronized)
{
    (void)address;
    (void)enabled;
    (void)synchronized;
    mock_send_count++;
    return 1u;
}

uint8 emm42_set_current_position_zero(uint8 address)
{
    (void)address;
    mock_send_count++;
    return 1u;
}

uint8 emm42_move_angle(uint8 address, float angle_deg, uint16 rpm,
                       uint8 acceleration, emm42_position_mode_enum mode,
                       uint8 synchronized)
{
    (void)address;
    (void)angle_deg;
    (void)rpm;
    (void)acceleration;
    (void)mode;
    (void)synchronized;
    mock_send_count++;
    mock_move_count++;
    return 1u;
}

uint8 emm42_query_position(uint8 address)
{
    (void)address;
    mock_send_count++;
    mock_position_query_count++;
    return 1u;
}

uint8 emm42_stop(uint8 address, uint8 synchronized)
{
    (void)address;
    (void)synchronized;
    mock_send_count++;
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

uint8 emm42_decode_ack(const emm42_frame_t *frame, uint8 address,
                       uint8 command, uint8 *status)
{
    if ((frame->length != 4u) || (frame->data[0] != address) ||
        (frame->data[1] != command))
    {
        return 0u;
    }
    *status = frame->data[2];
    return 1u;
}

uint8 emm42_decode_position_deg(const emm42_frame_t *frame, uint8 address,
                                float *position_deg)
{
    if ((frame->length != 8u) || (frame->data[0] != address) ||
        (frame->data[1] != 0x36u))
    {
        return 0u;
    }
    *position_deg = mock_position_deg;
    return 1u;
}

uint32 emm42_get_rx_overflow_count(void)
{
    return 0u;
}

void vision_link_get_status(vision_link_status_t *status)
{
    status->link_online = 0u;
}

uint8 vision_link_take_new_valid_measurement(vision_link_snapshot_t *snapshot)
{
    (void)snapshot;
    return 0u;
}

static void reset_mocks(void)
{
    mock_now_ms = 0u;
    mock_frame_ready = 0u;
    mock_send_count = 0u;
    mock_move_count = 0u;
    mock_position_query_count = 0u;
}

#if (BALANCE_STARTUP_CALIBRATED != 0u)
static void test_successful_startup(void)
{
    const balance_app_status_t *status;

    reset_mocks();
    balance_app_init();
    status = balance_app_get_status();
    assert(status->state == BALANCE_APP_POWER_WAIT);

    mock_now_ms = 3000u;
    balance_app_process();
    assert(balance_app_get_status()->state == BALANCE_APP_SET_REFERENCE);
    queue_ack(0x0Au);
    balance_app_process();
    assert(balance_app_get_status()->state == BALANCE_APP_ENABLE);
    queue_ack(0xF3u);
    balance_app_process();
    assert(balance_app_get_status()->state == BALANCE_APP_MOVE_LEVEL);
    queue_ack(0xFDu);
    balance_app_process();
    queue_position(level_motor_position());
    balance_app_process();
    mock_now_ms += 200u;
    balance_app_process();
    queue_position(level_motor_position());
    balance_app_process();
    assert(balance_app_get_status()->state == BALANCE_APP_ACTIVE);
}

static void test_command_timeouts_latch(void)
{
    reset_mocks();
    balance_app_init();
    mock_now_ms = 3000u;
    balance_app_process();
    mock_now_ms += 26u;
    balance_app_process();
    mock_now_ms += 26u;
    balance_app_process();
    mock_now_ms += 26u;
    balance_app_process();
    assert(balance_app_get_status()->state == BALANCE_APP_FAULT);
    assert(balance_app_get_status()->fault == BALANCE_FAULT_COMMAND_TIMEOUT);
}

static void test_active_position_query_and_move_run_at_100_hz(void)
{
    uint32 query_count;
    uint32 move_count;

    reset_mocks();
    balance_app_init();
    mock_now_ms = 3000u;
    balance_app_process();
    queue_ack(0x0Au);
    balance_app_process();
    queue_ack(0xF3u);
    balance_app_process();
    queue_ack(0xFDu);
    balance_app_process();
    queue_position(level_motor_position());
    balance_app_process();
    mock_now_ms += 200u;
    balance_app_process();
    queue_position(level_motor_position());
    balance_app_process();
    assert(balance_app_get_status()->state == BALANCE_APP_ACTIVE);

    query_count = mock_position_query_count;
    move_count = mock_move_count;
    mock_now_ms += BALANCE_POSITION_QUERY_PERIOD_MS;
    balance_app_process();
    assert(mock_move_count == move_count + 1u);

    queue_ack(0xFDu);
    balance_app_process();
    assert(mock_position_query_count == query_count + 1u);
    queue_position(level_motor_position());
    balance_app_process();

    mock_now_ms += BALANCE_POSITION_QUERY_PERIOD_MS;
    balance_app_process();
    assert(mock_move_count == move_count + 2u);
    queue_ack(0xFDu);
    balance_app_process();
    assert(mock_position_query_count == query_count + 2u);
}
#else
static void test_unconfigured_mode_queries_without_motion(void)
{
    reset_mocks();
    balance_app_init();
    assert(balance_app_get_status()->state == BALANCE_APP_UNCONFIGURED);
    mock_now_ms = BALANCE_POSITION_QUERY_PERIOD_MS;
    balance_app_process();
    assert(mock_position_query_count == 1u);
    assert(mock_move_count == 0u);
}
#endif

int main(void)
{
#if (BALANCE_STARTUP_CALIBRATED != 0u)
    test_successful_startup();
    test_command_timeouts_latch();
    test_active_position_query_and_move_run_at_100_hz();
#else
    test_unconfigured_mode_queries_without_motion();
#endif
    puts("balance app tests passed");
    return 0;
}
