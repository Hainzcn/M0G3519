#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "balance_linkage.h"
#include "button.h"
#include "emm42.h"
#include "emm42_demo_app.h"
#include "vision_link.h"

static uint32 mock_now_ms;
static emm42_frame_t mock_frame;
static uint8 mock_frame_ready;
static float mock_position_deg;
static float mock_last_move_deg;
static uint32 mock_move_count;
static uint32 mock_stop_count;
static uint32 mock_query_count;
static button_id_t mock_button;
static uint8 mock_vision_valid;
static vision_link_snapshot_t mock_vision;

static float expected_motor_position(float physical_lever_deg)
{
    float motor_angle_deg;

    assert(0u != balance_linkage_motor_from_physical_lever_deg(
        physical_lever_deg, &motor_angle_deg));
    return motor_angle_deg;
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

uint8 vision_link_get_valid_measurement(vision_link_snapshot_t *snapshot)
{
    if (0u == mock_vision_valid)
    {
        return 0u;
    }
    *snapshot = mock_vision;
    return 1u;
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

static void press_button(button_id_t button, uint32 now_ms)
{
    mock_button = button;
    process_at(now_ms);
    mock_button = BUTTON_ID_NONE;
    process_at(now_ms + 1u);
}

int main(void)
{
    float first_target;

    mock_now_ms = 0u;
    mock_frame_ready = 0u;
    mock_move_count = 0u;
    mock_stop_count = 0u;
    mock_query_count = 0u;
    mock_button = BUTTON_ID_NONE;
    mock_vision_valid = 1u;
    mock_vision.flags = VISION_LINK_FLAG_MEASURED_VALID |
                        VISION_LINK_FLAG_TRACKER_READY |
                        VISION_LINK_FLAG_CALIBRATION_VALID;
    mock_vision.confidence = 80u;
    emm42_demo_app_init();
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_POWER);
    assert(emm42_demo_app_get_target_lever_deg() == 0.0f);

    process_at(3000u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_ZERO);
    process_at(3100u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_ENABLE);
    process_at(3200u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_MOVE_ANGLE);
    process_at(3200u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_ANGLE);
    assert(mock_move_count == 1u);
    first_target = expected_motor_position(0.0f);
    assert(fabsf(mock_last_move_deg - first_target) < 0.001f);

    process_at(3220u);
    assert(mock_query_count == 1u);
    queue_position(first_target);
    process_at(3220u);
    process_at(4200u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_READY);
    assert(mock_query_count == 2u);

    press_button(BUTTON_ID_SW1, 4210u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_RECORDING);
    assert(emm42_demo_app_get_trial_id() == 1u);
    assert(emm42_demo_app_is_recording() != 0u);
    process_at(4230u);
    assert(mock_query_count == 3u);
    process_at(8210u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_READY);

    mock_vision_valid = 0u;
    press_button(BUTTON_ID_SW1, 8215u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_READY);
    assert(emm42_demo_app_get_trial_id() == 1u);
    mock_vision_valid = 1u;

    press_button(BUTTON_ID_SW2, 8220u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_WAIT_ANGLE);
    assert(emm42_demo_app_get_target_lever_deg() == -2.0f);
    assert(mock_move_count == 2u);

    press_button(BUTTON_ID_SW4, 8230u);
    assert(emm42_demo_app_get_state() == EMM42_DEMO_ERROR);
    assert(mock_stop_count == 1u);

    puts("emm42 ball dynamics demo tests passed");
    return 0;
}
