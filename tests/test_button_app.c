#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ab_run_app.h"
#include "balance_app.h"
#include "balance_simple_app.h"
#include "button.h"
#include "button_app.h"
#include "control_config.h"
#include "drive_balance_demo_app.h"
#include "motor_app.h"
#include "no_load_lap_app.h"
#include "oled.h"
#include "stop_test_app.h"
#include "vision_link.h"

static button_id_t mock_button;
static uint32 mock_now_ms;
static uint8 mock_oled_ready;
static uint8 mock_balance_start_result;
static uint8 mock_balance_simple_start_result;
static uint8 mock_drive_start_result;
static uint8 mock_drive_running;
static drive_balance_demo_status_t mock_drive_status;
static uint8 mock_no_load_start_result;
static uint8 mock_no_load_running;
static uint8 mock_ab_start_result;
static uint8 mock_ab_running;
static ab_run_status_t mock_ab_status;
static uint8 mock_stop_test_start_result;
static uint8 mock_stop_test_running;
static stop_test_status_t mock_stop_test_status;
static no_load_lap_status_t mock_no_load_status;
static const char *mock_oled_title;
static uint32 mock_dashboard_disable_count;
static uint32 mock_refresh_count;
static uint32 mock_motor_line_start_count;
static uint32 mock_motor_stop_count;
static uint32 mock_balance_start_count;
static uint32 mock_balance_cancel_count;
static uint32 mock_balance_simple_start_count;
static uint32 mock_balance_simple_disable_count;
static uint32 mock_drive_center_start_count;
static uint32 mock_drive_capture_prepare_count;
static uint32 mock_drive_captured_start_count;
static uint32 mock_drive_stop_count;
static uint8 mock_drive_capture_ready;
static uint32 mock_no_load_start_count;
static uint32 mock_no_load_stop_count;
static uint32 mock_ab_start_count;
static uint32 mock_ab_stop_count;
static uint32 mock_stop_test_start_count;
static uint32 mock_stop_test_stop_count;
static uint8 mock_vision_online;
static uint8 mock_oled_vision_off;
static uint8 mock_oled_post_marker;
static float mock_fixed_beam_bias_deg;
static float mock_vision_position_offset_m;
static uint8 mock_vision_has_snapshot;
static vision_link_snapshot_t mock_vision_snapshot;

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void button_init(void)
{
}

void button_process(void)
{
}

button_id_t button_get_active(void)
{
    return mock_button;
}

uint8 oled_is_ready(void)
{
    return mock_oled_ready;
}

void oled_clear(void)
{
    mock_oled_vision_off = 0u;
    mock_oled_post_marker = 0u;
}

void oled_show_char(uint8 x, uint8 page, char chr, uint8 font)
{
    (void)x;
    (void)page;
    (void)chr;
    (void)font;
}

void oled_show_string(uint8 x, uint8 page, const char *text, uint8 font)
{
    (void)font;
    if ((0u == x) && (0u == page))
    {
        mock_oled_title = text;
    }
    if ((0u == x) && (4u == page) &&
        (0 == strcmp(text, "VISION OFF")))
    {
        mock_oled_vision_off = 1u;
    }
    if ((0u == x) && (4u == page) &&
        (0 == strcmp(text, "POST:")))
    {
        mock_oled_post_marker = 1u;
    }
}

void vision_link_get_status(vision_link_status_t *status)
{
    memset(status, 0, sizeof(*status));
    status->link_online = mock_vision_online;
}

uint8 vision_link_get_latest_snapshot(vision_link_snapshot_t *snapshot)
{
    if (0u == mock_vision_has_snapshot)
    {
        return 0u;
    }
    *snapshot = mock_vision_snapshot;
    return 1u;
}

void vision_link_set_position_offset_m(float offset_m)
{
    if (offset_m > VISION_LINK_POSITION_OFFSET_LIMIT_M)
    {
        offset_m = VISION_LINK_POSITION_OFFSET_LIMIT_M;
    }
    else if (offset_m < -VISION_LINK_POSITION_OFFSET_LIMIT_M)
    {
        offset_m = -VISION_LINK_POSITION_OFFSET_LIMIT_M;
    }
    mock_vision_position_offset_m = offset_m;
}

float vision_link_get_position_offset_m(void)
{
    return mock_vision_position_offset_m;
}

void oled_show_uint(uint8 x, uint8 page, uint32 value, uint8 font)
{
    (void)x;
    (void)page;
    (void)value;
    (void)font;
}

void oled_refresh(void)
{
    mock_refresh_count++;
}

void oled_app_set_dashboard_enabled(uint8 enabled)
{
    if (0u == enabled)
    {
        mock_dashboard_disable_count++;
    }
}

void heartbeat_hw_uart_send_string(const char *message)
{
    (void)message;
}

void motor_app_set_line_follow_enabled(uint8 enabled)
{
    if (0u != enabled)
    {
        mock_motor_line_start_count++;
    }
}

void motor_app_stop(void)
{
    mock_motor_stop_count++;
}

uint8 balance_app_start_sequence(void)
{
    mock_balance_start_count++;
    return mock_balance_start_result;
}

void balance_app_cancel_motion(void)
{
    mock_balance_cancel_count++;
}

uint8 balance_simple_app_start(void)
{
    mock_balance_simple_start_count++;
    return mock_balance_simple_start_result;
}

void balance_simple_app_disable(void)
{
    mock_balance_simple_disable_count++;
}

uint8 drive_balance_demo_app_start_center(void)
{
    mock_drive_center_start_count++;
    mock_drive_running = mock_drive_start_result;
    return mock_drive_start_result;
}

uint8 drive_balance_demo_app_start_captured(void)
{
    mock_drive_captured_start_count++;
    mock_drive_running = (0u != mock_drive_capture_ready) ?
        mock_drive_start_result : 0u;
    return mock_drive_running;
}

void balance_simple_app_set_fixed_beam_bias_deg(float bias_deg)
{
    if (bias_deg > BALANCE_SIMPLE_MAX_TARGET_BEAM_ANGLE_DEG)
    {
        bias_deg = BALANCE_SIMPLE_MAX_TARGET_BEAM_ANGLE_DEG;
    }
    if (bias_deg < -BALANCE_SIMPLE_MAX_TARGET_BEAM_ANGLE_DEG)
    {
        bias_deg = -BALANCE_SIMPLE_MAX_TARGET_BEAM_ANGLE_DEG;
    }
    mock_fixed_beam_bias_deg = bias_deg;
}

float balance_simple_app_get_fixed_beam_bias_deg(void)
{
    return mock_fixed_beam_bias_deg;
}

void drive_balance_demo_app_stop(void)
{
    mock_drive_stop_count++;
    mock_drive_running = 0u;
}

uint8 drive_balance_demo_app_is_running(void)
{
    return mock_drive_running;
}

const drive_balance_demo_status_t *drive_balance_demo_app_get_status(void)
{
    return &mock_drive_status;
}

uint8 no_load_lap_app_start(void)
{
    mock_no_load_start_count++;
    mock_no_load_running = mock_no_load_start_result;
    mock_no_load_status.state = (0u != mock_no_load_start_result) ?
        NO_LOAD_LAP_RUNNING : NO_LOAD_LAP_IDLE;
    return mock_no_load_start_result;
}

void no_load_lap_app_stop(void)
{
    mock_no_load_stop_count++;
    mock_no_load_running = 0u;
    mock_no_load_status.state = NO_LOAD_LAP_USER_STOP;
}

uint8 no_load_lap_app_is_running(void)
{
    return mock_no_load_running;
}

const no_load_lap_status_t *no_load_lap_app_get_status(void)
{
    return &mock_no_load_status;
}

uint8 drive_balance_demo_app_prepare_captured(void)
{
    mock_drive_capture_prepare_count++;
    return mock_drive_start_result;
}

uint8 drive_balance_demo_app_capture_ready(void)
{
    return mock_drive_capture_ready;
}

uint8 stop_test_app_start(void)
{
    mock_stop_test_start_count++;
    mock_stop_test_running = mock_stop_test_start_result;
    mock_stop_test_status.state = (0u != mock_stop_test_start_result) ?
        STOP_TEST_TO_POSITIVE : STOP_TEST_IDLE;
    return mock_stop_test_start_result;
}

void stop_test_app_stop(void)
{
    mock_stop_test_stop_count++;
    mock_stop_test_running = 0u;
    mock_stop_test_status.state = STOP_TEST_USER_STOP;
}

uint8 stop_test_app_is_running(void)
{
    return mock_stop_test_running;
}

const stop_test_status_t *stop_test_app_get_status(void)
{
    return &mock_stop_test_status;
}

uint8 ab_run_app_start(void)
{
    mock_ab_start_count++;
    mock_ab_running = mock_ab_start_result;
    mock_ab_status.state = (0u != mock_ab_start_result) ?
        AB_RUN_RUNNING : AB_RUN_IDLE;
    return mock_ab_start_result;
}

void ab_run_app_stop(void)
{
    mock_ab_stop_count++;
    mock_ab_running = 0u;
    mock_ab_status.state = AB_RUN_USER_STOP;
}

uint8 ab_run_app_is_running(void)
{
    return mock_ab_running;
}

const ab_run_status_t *ab_run_app_get_status(void)
{
    return &mock_ab_status;
}

static void reset_mocks(void)
{
    mock_button = BUTTON_ID_NONE;
    mock_now_ms = 0u;
    mock_oled_ready = 1u;
    mock_balance_start_result = 1u;
    mock_balance_simple_start_result = 1u;
    mock_drive_start_result = 1u;
    mock_drive_running = 0u;
    memset(&mock_drive_status, 0, sizeof(mock_drive_status));
    mock_no_load_start_result = 1u;
    mock_no_load_running = 0u;
    mock_ab_start_result = 1u;
    mock_ab_running = 0u;
    memset(&mock_ab_status, 0, sizeof(mock_ab_status));
    mock_stop_test_start_result = 1u;
    mock_stop_test_running = 0u;
    memset(&mock_stop_test_status, 0, sizeof(mock_stop_test_status));
    mock_no_load_status.state = NO_LOAD_LAP_IDLE;
    mock_no_load_status.elapsed_ms = 0u;
    mock_no_load_status.distance_m = 0.0f;
    mock_oled_title = NULL;
    mock_dashboard_disable_count = 0u;
    mock_refresh_count = 0u;
    mock_motor_line_start_count = 0u;
    mock_motor_stop_count = 0u;
    mock_balance_start_count = 0u;
    mock_balance_cancel_count = 0u;
    mock_balance_simple_start_count = 0u;
    mock_balance_simple_disable_count = 0u;
    mock_drive_center_start_count = 0u;
    mock_drive_capture_prepare_count = 0u;
    mock_drive_captured_start_count = 0u;
    mock_drive_stop_count = 0u;
    mock_drive_capture_ready = 1u;
    mock_no_load_start_count = 0u;
    mock_no_load_stop_count = 0u;
    mock_ab_start_count = 0u;
    mock_ab_stop_count = 0u;
    mock_stop_test_start_count = 0u;
    mock_stop_test_stop_count = 0u;
    mock_vision_online = 1u;
    mock_oled_vision_off = 0u;
    mock_oled_post_marker = 0u;
    mock_fixed_beam_bias_deg = BALANCE_SIMPLE_FIXED_BEAM_BIAS_DEG;
    mock_vision_position_offset_m = BALANCE_VISION_POSITION_OFFSET_M;
    mock_vision_has_snapshot = 0u;
    memset(&mock_vision_snapshot, 0, sizeof(mock_vision_snapshot));
    button_app_init();
    button_app_process();
}

static void press_button(button_id_t button)
{
    mock_button = button;
    button_app_process();
    mock_button = BUTTON_ID_NONE;
    button_app_process();
}

static void confirm_selected_mode(void)
{
    press_button(BUTTON_ID_SW3);
    press_button(BUTTON_ID_SW3);
}

static void test_navigation_wraps(void)
{
    reset_mocks();
    assert(mock_dashboard_disable_count == 1u);
    assert(button_app_get_selected_mode() == BUTTON_APP_MODE_NO_LOAD);
    press_button(BUTTON_ID_SW2);
    assert(button_app_get_selected_mode() == BUTTON_APP_MODE_ARBITRARY);
    press_button(BUTTON_ID_SW1);
    assert(button_app_get_selected_mode() == BUTTON_APP_MODE_NO_LOAD);
}

static void test_no_load_start_and_stop(void)
{
    reset_mocks();
    confirm_selected_mode();
    assert(0u != button_app_is_running());
    assert(mock_no_load_start_count == 1u);
#if (BALANCE_CONTROL_ENABLE != 0u)
    assert(mock_balance_cancel_count == 1u);
#endif
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
    assert(mock_balance_simple_disable_count == 0u);
#endif
    press_button(BUTTON_ID_SW4);
    assert(0u == button_app_is_running());
    assert(mock_no_load_stop_count == 1u);
}

static void test_no_load_completion_holds_result_page(void)
{
    reset_mocks();
    confirm_selected_mode();
    mock_no_load_running = 0u;
    mock_no_load_status.state = NO_LOAD_LAP_COMPLETE;
    mock_no_load_status.elapsed_ms = 12345u;
    mock_no_load_status.distance_m = 6.14f;
    button_app_process();
    assert(0u == button_app_is_running());
    assert(0 == strcmp(mock_oled_title, "LAP COMPLETE"));

    press_button(BUTTON_ID_SW4);
    assert(0 == strcmp(mock_oled_title, "TRACK MODE SELECT"));
}

static void test_ab_start_and_completion(void)
{
    reset_mocks();
    mock_vision_online = 0u;
    press_button(BUTTON_ID_SW1);
    press_button(BUTTON_ID_SW1);
    assert(button_app_get_selected_mode() == BUTTON_APP_MODE_AB);
    confirm_selected_mode();
    assert(0u != button_app_is_running());
    assert(mock_ab_start_count == 1u);
    assert(mock_no_load_start_count == 0u);
    assert(mock_drive_center_start_count == 0u);
    assert(mock_oled_vision_off != 0u);

    mock_ab_running = 0u;
    mock_ab_status.state = AB_RUN_COMPLETE;
    mock_ab_status.elapsed_ms = 3200u;
    mock_ab_status.max_abs_error_m = 0.008f;
    mock_ab_status.error_requirement_met = 1u;
    button_app_process();
    assert(0u == button_app_is_running());
    assert(0 == strcmp(mock_oled_title, "AB COMPLETE"));
    press_button(BUTTON_ID_SW4);
    assert(button_app_get_selected_mode() == BUTTON_APP_MODE_AB);
}

static void test_stop_test_start_completion_and_stop(void)
{
    reset_mocks();
    press_button(BUTTON_ID_SW1);
    confirm_selected_mode();
    assert(0u != button_app_is_running());
    assert(mock_stop_test_start_count == 1u);
    assert(mock_balance_start_count == 0u);
    assert(mock_balance_simple_start_count == 0u);

    mock_stop_test_running = 0u;
    mock_stop_test_status.state = STOP_TEST_COMPLETE;
    mock_stop_test_status.elapsed_ms = 4200u;
    mock_stop_test_status.positive_max_abs_error_m = 0.006f;
    mock_stop_test_status.negative_max_abs_error_m = 0.007f;
    mock_stop_test_status.overall_requirement_met = 1u;
    button_app_process();
    assert(0u == button_app_is_running());
    assert(0 == strcmp(mock_oled_title, "STOP TEST OK"));
    press_button(BUTTON_ID_SW4);
    assert(mock_stop_test_stop_count == 1u);

    confirm_selected_mode();
    assert(0u != button_app_is_running());
    press_button(BUTTON_ID_SW4);
    assert(mock_stop_test_stop_count == 2u);
}

#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
static void test_long_sw1_opens_and_adjusts_bias(void)
{
    reset_mocks();
    mock_button = BUTTON_ID_SW1;
    button_app_process();
    mock_now_ms = BUTTON_APP_TUNING_LONG_PRESS_MS;
    button_app_process();
    assert(0 == strcmp(mock_oled_title, "BEAM BIAS TUNE"));
    assert(button_app_get_selected_mode() == BUTTON_APP_MODE_NO_LOAD);

    mock_button = BUTTON_ID_NONE;
    button_app_process();
    press_button(BUTTON_ID_SW1);
    assert(mock_fixed_beam_bias_deg ==
           BALANCE_SIMPLE_FIXED_BEAM_BIAS_DEG - 0.2f);
    press_button(BUTTON_ID_SW2);
    press_button(BUTTON_ID_SW2);
    assert(mock_fixed_beam_bias_deg ==
           BALANCE_SIMPLE_FIXED_BEAM_BIAS_DEG + 0.2f);

    press_button(BUTTON_ID_SW4);
    assert(0 == strcmp(mock_oled_title, "TRACK MODE SELECT"));
}
#endif

static void test_long_sw2_opens_and_adjusts_vision_offset(void)
{
    reset_mocks();
    mock_vision_has_snapshot = 1u;
    mock_vision_snapshot.position_dmm = 80;
    mock_button = BUTTON_ID_SW2;
    button_app_process();
    mock_now_ms = BUTTON_APP_TUNING_LONG_PRESS_MS;
    button_app_process();
    assert(0 == strcmp(mock_oled_title, "VISION POS OFFSET"));
    assert(button_app_get_selected_mode() == BUTTON_APP_MODE_NO_LOAD);

    mock_button = BUTTON_ID_NONE;
    button_app_process();
    press_button(BUTTON_ID_SW1);
    assert(mock_vision_position_offset_m ==
           BALANCE_VISION_POSITION_OFFSET_M - 0.002f);
    press_button(BUTTON_ID_SW2);
    press_button(BUTTON_ID_SW2);
    assert(mock_vision_position_offset_m ==
           BALANCE_VISION_POSITION_OFFSET_M + 0.002f);

    press_button(BUTTON_ID_SW4);
    assert(0 == strcmp(mock_oled_title, "TRACK MODE SELECT"));
}

static void test_no_load_marker_state_refreshes_running_page(void)
{
    uint32 refresh_count;

    reset_mocks();
    confirm_selected_mode();
    refresh_count = mock_refresh_count;

    mock_no_load_status.state = NO_LOAD_LAP_POST_MARKER;
    mock_no_load_status.brake_distance_m = 0.001f;
    button_app_process();
    assert(mock_refresh_count == (refresh_count + 1u));
    assert(0u != mock_oled_post_marker);

    refresh_count = mock_refresh_count;
    mock_no_load_status.brake_distance_m = 0.009f;
    button_app_process();
    assert(mock_refresh_count == refresh_count);

    mock_no_load_status.brake_distance_m = 0.011f;
    button_app_process();
    assert(mock_refresh_count == (refresh_count + 1u));
    assert(0u != mock_oled_post_marker);
}

#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
static void test_drive_mode_completion_shows_result(void)
{
    reset_mocks();
    mock_vision_online = 0u;
    press_button(BUTTON_ID_SW1);
    press_button(BUTTON_ID_SW1);
    press_button(BUTTON_ID_SW1);
    assert(button_app_get_selected_mode() == BUTTON_APP_MODE_BALL_LAP);
    confirm_selected_mode();
    assert(0u != button_app_is_running());
    assert(mock_drive_center_start_count == 1u);
    assert(mock_oled_vision_off != 0u);
    mock_drive_running = 0u;
    mock_drive_status.state = DRIVE_BALANCE_DEMO_COMPLETE;
    mock_drive_status.error_requirement_met = 1u;
    mock_drive_status.elapsed_ms = 15000u;
    mock_drive_status.max_abs_error_m = 0.009f;
    button_app_process();
    assert(0u == button_app_is_running());
    assert(0 == strcmp(mock_oled_title, "BALL LAP OK"));

    press_button(BUTTON_ID_SW4);

    press_button(BUTTON_ID_SW1);
    mock_drive_capture_ready = 0u;
    confirm_selected_mode();
    assert(0u == button_app_is_running());
    assert(mock_drive_capture_prepare_count == 1u);
    assert(mock_drive_captured_start_count == 0u);
    press_button(BUTTON_ID_SW3);
    assert(0u == button_app_is_running());
    assert(mock_drive_captured_start_count == 1u);
    mock_drive_capture_ready = 1u;
    button_app_process();
    press_button(BUTTON_ID_SW3);
    assert(0u != button_app_is_running());
    assert(mock_drive_captured_start_count == 2u);
    assert(mock_oled_vision_off != 0u);
    press_button(BUTTON_ID_SW4);
    assert(mock_drive_stop_count == 1u);
    assert(0u == button_app_is_running());
}
#endif

int main(void)
{
    test_navigation_wraps();
    test_long_sw2_opens_and_adjusts_vision_offset();
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
    test_long_sw1_opens_and_adjusts_bias();
#endif
    test_no_load_start_and_stop();
    test_no_load_completion_holds_result_page();
    test_no_load_marker_state_refreshes_running_page();
    test_ab_start_and_completion();
    test_stop_test_start_completion_and_stop();
#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
    test_drive_mode_completion_shows_result();
#endif
    puts("button app tests passed");
    return 0;
}
