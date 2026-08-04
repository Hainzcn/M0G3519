#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "ball_return_demo_app.h"
#include "balance_linkage.h"
#include "button.h"
#include "control_config.h"
#include "control_config_legacy.h"
#include "emm42.h"

static uint32 mock_now_ms;
static emm42_frame_t mock_frame;
static uint8 mock_frame_ready;
static float mock_position_deg;
static float mock_last_move_deg;
static uint32 mock_move_count;
static uint32 mock_stop_count;
static uint32 mock_query_count;
static uint32 mock_zero_count;
static uint32 mock_enable_count;
static button_id_t mock_button;

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void heartbeat_hw_uart_send_string(const char *message)
{
    (void)message;
}

button_id_t button_get_active(void)
{
    return mock_button;
}

void emm42_init(void)
{
}

uint8 emm42_set_current_position_zero(uint8 address)
{
    (void)address;
    mock_zero_count++;
    return 1u;
}

uint8 emm42_set_enabled(uint8 address, uint8 enabled, uint8 synchronized)
{
    (void)address;
    (void)enabled;
    (void)synchronized;
    mock_enable_count++;
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
    mock_stop_count++;
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
    if (0u == mock_frame_ready) return 0u;
    *frame = mock_frame;
    mock_frame_ready = 0u;
    return 1u;
}

uint8 emm42_decode_position_deg(const emm42_frame_t *frame, uint8 address,
                                float *position_deg)
{
    if ((8u != frame->length) || (address != frame->data[0]) ||
        (0x36u != frame->data[1]))
        return 0u;
    *position_deg = mock_position_deg;
    return 1u;
}

static void queue_position(float position_deg)
{
    mock_position_deg = position_deg;
    mock_frame.data[0] = EMM42_DEFAULT_ADDRESS;
    mock_frame.data[1] = 0x36u;
    mock_frame.data[7] = 0x6Bu;
    mock_frame.length = 8u;
    mock_frame_ready = 1u;
}

static void process_at(uint32 now_ms)
{
    mock_now_ms = now_ms;
    ball_return_demo_app_process();
}

static void press_button(button_id_t button, uint32 now_ms)
{
    mock_button = button;
    process_at(now_ms);
    mock_button = BUTTON_ID_NONE;
    process_at(now_ms + 1u);
}

static uint32 bring_demo_ready(void)
{
    float level_motor_deg;

    mock_now_ms = 0u;
    mock_frame_ready = 0u;
    mock_move_count = 0u;
    mock_stop_count = 0u;
    mock_query_count = 0u;
    mock_zero_count = 0u;
    mock_enable_count = 0u;
    mock_button = BUTTON_ID_NONE;
    ball_return_demo_app_init();

    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_WAIT_POWER);
    process_at(2999u);
    assert(mock_zero_count == 0u);
    process_at(3000u);
    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_WAIT_ZERO);
    assert(mock_zero_count == 1u);
    process_at(3100u);
    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_WAIT_ENABLE);
    assert(mock_enable_count == 1u);
    process_at(3200u);
    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_MOVE_LEVEL);
    process_at(3200u);
    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_WAIT_LEVEL);
    assert(mock_move_count == 1u);
    assert(0u != balance_linkage_motor_from_physical_lever_deg(
        0.0f, &level_motor_deg));
    assert(fabsf(mock_last_move_deg - level_motor_deg) < 0.001f);

    process_at(3220u);
    assert(mock_query_count == 1u);
    queue_position(level_motor_deg);
    process_at(3220u);
    process_at(3419u);
    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_WAIT_LEVEL);
    queue_position(level_motor_deg);
    process_at(3420u);
    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_READY);
    return 3420u;
}

static void test_return_run_and_repeat(void)
{
    uint32 now_ms = bring_demo_ready();
    uint32 running_query_count;
    float previous_position;
    uint8 saw_positive_raw_angle = 0u;
    uint8 saw_nonzero_shaped_angle = 0u;
    uint32 step;

    press_button(BUTTON_ID_SW1, now_ms + 10u);
    now_ms += 11u;
    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_RUNNING);
    assert(fabsf(ball_return_demo_app_get_reference_position_m() - 0.050f) <
           0.000001f);
    assert(fabsf(ball_return_demo_app_get_reference_velocity_mps()) <
           0.000001f);
    running_query_count = mock_query_count;
    previous_position = ball_return_demo_app_get_reference_position_m();

    for (step = 0u; step < 2000u; step++)
    {
        float position;
        float raw_angle;
        float shaped_angle;

        now_ms += BALANCE_ESTIMATOR_PERIOD_MS;
        process_at(now_ms);
        position = ball_return_demo_app_get_reference_position_m();
        raw_angle = ball_return_demo_app_get_raw_lever_angle_deg();
        shaped_angle = ball_return_demo_app_get_lever_angle_deg();
        assert(position <= previous_position + 0.000001f);
        assert(position >= -0.000001f);
        assert(fabsf(raw_angle) <= BALANCE_MAX_LEVER_ANGLE_DEG + 0.0001f);
        assert(fabsf(shaped_angle) <= BALANCE_MAX_LEVER_ANGLE_DEG + 0.0001f);
        if (raw_angle > 0.01f) saw_positive_raw_angle = 1u;
        if (fabsf(shaped_angle) > 0.01f) saw_nonzero_shaped_angle = 1u;
        previous_position = position;
        if (ball_return_demo_app_get_state() == BALL_RETURN_DEMO_DONE) break;
    }

    assert(saw_positive_raw_angle != 0u);
    assert(saw_nonzero_shaped_angle != 0u);
    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_DONE);
    assert(ball_return_demo_app_get_motion_phase() == BALL_MOTION_PHASE_CAPTURE ||
           ball_return_demo_app_get_motion_phase() == BALL_MOTION_PHASE_HOLD);
    assert(mock_query_count == running_query_count);
    assert(fabsf(ball_return_demo_app_get_lever_angle_deg()) < 0.001f);

    press_button(BUTTON_ID_SW1, now_ms + 10u);
    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_RUNNING);
    assert(fabsf(ball_return_demo_app_get_reference_position_m() - 0.050f) <
           0.000001f);
    press_button(BUTTON_ID_SW4, now_ms + 20u);
    assert(ball_return_demo_app_get_state() == BALL_RETURN_DEMO_ERROR);
    assert(mock_stop_count == 1u);
}

int main(void)
{
    test_return_run_and_repeat();
    puts("ball return demo tests passed");
    return 0;
}
