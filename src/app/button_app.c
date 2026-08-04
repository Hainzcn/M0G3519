#include "button_app.h"

#include "ab_run_app.h"
#include "button.h"
#include "control_config.h"
#include "heartbeat_hw.h"
#include "motor_app.h"
#include "no_load_lap_app.h"
#include "oled.h"
#include "oled_app.h"
#include "stop_test_app.h"
#include "vision_link.h"
#if (BALANCE_CONTROL_ENABLE != 0u)
#include "balance_app.h"
#endif
#if (BALANCE_SIMPLE_CONTROL_ENABLE != 0u)
#include "balance_simple_app.h"
#endif
#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
#include "drive_balance_demo_app.h"
#endif

#define BUTTON_APP_MODE_COUNT             (5u)
#define BUTTON_APP_MODE_FIRST_PAGE        (1u)

typedef enum
{
    BUTTON_APP_VIEW_MENU = 0,
    BUTTON_APP_VIEW_CONFIRM,
    BUTTON_APP_VIEW_CAPTURE,
    BUTTON_APP_VIEW_RUNNING,
    BUTTON_APP_VIEW_REJECTED,
    BUTTON_APP_VIEW_RESULT,
} button_app_view_enum;

static const char *const button_app_mode_names[BUTTON_APP_MODE_COUNT] =
{
    "NO LOAD",
    "STOP TEST",
    "A-B RUN",
    "BALL LAP",
    "ANY POSITION",
};

static button_id_t button_app_previous;
static button_app_mode_enum button_app_selected_mode;
static button_app_mode_enum button_app_running_mode;
static button_app_view_enum button_app_view;
static uint8 button_app_force_render;
static uint8 button_app_last_vision_online;
static no_load_lap_state_enum button_app_last_no_load_state;
static uint32 button_app_last_post_distance_step;
static uint8 button_app_last_capture_ready;

static uint8 button_app_mode_uses_vision(button_app_mode_enum mode)
{
    return ((BUTTON_APP_MODE_AB == mode) ||
            (BUTTON_APP_MODE_BALL_LAP == mode) ||
            (BUTTON_APP_MODE_ARBITRARY == mode)) ? 1u : 0u;
}

static void button_app_show_mode(uint8 page, button_app_mode_enum mode,
                                 uint8 selected)
{
    oled_show_char(0u, page, (0u != selected) ? '>' : ' ', OLED_FONT_6X8);
    oled_show_uint(6u, page, (uint32)mode + 1u, OLED_FONT_6X8);
    oled_show_char(12u, page, ' ', OLED_FONT_6X8);
    oled_show_string(18u, page, button_app_mode_names[(uint8)mode],
                     OLED_FONT_6X8);
}

static void button_app_render_menu(void)
{
    uint8 index;

    oled_clear();
    oled_show_string(0u, 0u, "TRACK MODE SELECT", OLED_FONT_6X8);
    for (index = 0u; index < BUTTON_APP_MODE_COUNT; index++)
    {
        button_app_show_mode(
            (uint8)(BUTTON_APP_MODE_FIRST_PAGE + index),
            (button_app_mode_enum)index,
            (index == (uint8)button_app_selected_mode) ? 1u : 0u);
    }
    oled_show_string(0u, 7u, "SW1 DN SW2 UP SW3 OK", OLED_FONT_6X8);
}

static void button_app_render_dialog(const char *title, const char *message)
{
    oled_clear();
    oled_show_string(0u, 0u, title, OLED_FONT_6X8);
    button_app_show_mode(2u, button_app_selected_mode, 1u);
    oled_show_string(0u, 4u, message, OLED_FONT_6X8);
    oled_show_string(0u, 6u, "SW3 YES", OLED_FONT_6X8);
    oled_show_string(66u, 6u, "SW4 BACK", OLED_FONT_6X8);
}

static void button_app_render_running(void)
{
    vision_link_status_t vision_status;

    oled_clear();
    oled_show_string(0u, 0u,
        (BUTTON_APP_MODE_NO_LOAD == button_app_running_mode) ?
        "LAP IN PROGRESS" : "MODE RUNNING", OLED_FONT_6X8);
    button_app_show_mode(2u, button_app_running_mode, 1u);
    if (BUTTON_APP_MODE_NO_LOAD == button_app_running_mode)
    {
        const no_load_lap_status_t *status =
            no_load_lap_app_get_status();

        if (NO_LOAD_LAP_POST_MARKER == status->state)
        {
            oled_show_string(0u, 4u, "POST:", OLED_FONT_6X8);
            oled_show_uint(30u, 4u,
                (uint32)(status->brake_distance_m * 1000.0f),
                OLED_FONT_6X8);
            oled_show_string(66u, 4u, "/210mm", OLED_FONT_6X8);
        }
        else
        {
            oled_show_string(0u, 4u, "SEARCH A", OLED_FONT_6X8);
        }
    }
    else if (0u != button_app_mode_uses_vision(button_app_running_mode))
    {
        vision_link_get_status(&vision_status);
        if (0u == vision_status.link_online)
        {
            oled_show_string(0u, 4u, "VISION OFF", OLED_FONT_6X8);
        }
    }
    oled_show_string(0u, 5u, "SW4 STOP / BACK", OLED_FONT_6X8);
}

#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
static void button_app_render_capture(void)
{
    vision_link_status_t vision_status;

    oled_clear();
    oled_show_string(0u, 0u, "SET BALL POSITION", OLED_FONT_6X8);
    button_app_show_mode(2u, BUTTON_APP_MODE_ARBITRARY, 1u);
    vision_link_get_status(&vision_status);
    if (0u == vision_status.link_online)
    {
        oled_show_string(0u, 4u, "VISION OFF", OLED_FONT_6X8);
    }
    else if (0u != drive_balance_demo_app_capture_ready())
    {
        oled_show_string(0u, 4u, "SW3 CAPTURE + RUN", OLED_FONT_6X8);
    }
    else
    {
        oled_show_string(0u, 4u, "LEVELING / WAIT", OLED_FONT_6X8);
    }
    oled_show_string(0u, 6u, "SW4 BACK", OLED_FONT_6X8);
}
#endif

static void button_app_check_vision_status(void)
{
    vision_link_status_t vision_status;

    if ((BUTTON_APP_VIEW_RUNNING != button_app_view) ||
        (0u == button_app_mode_uses_vision(button_app_running_mode)))
    {
        button_app_last_vision_online = 2u;
        return;
    }
    vision_link_get_status(&vision_status);
    if (button_app_last_vision_online != vision_status.link_online)
    {
        button_app_last_vision_online = vision_status.link_online;
        button_app_force_render = 1u;
    }
}

static void button_app_check_no_load_status(void)
{
    const no_load_lap_status_t *status;
    uint32 post_distance_step;

    if ((BUTTON_APP_VIEW_RUNNING != button_app_view) ||
        (BUTTON_APP_MODE_NO_LOAD != button_app_running_mode))
    {
        button_app_last_no_load_state = NO_LOAD_LAP_IDLE;
        button_app_last_post_distance_step = 0u;
        return;
    }

    status = no_load_lap_app_get_status();
    if (button_app_last_no_load_state != status->state)
    {
        button_app_last_no_load_state = status->state;
        button_app_force_render = 1u;
    }
    if (NO_LOAD_LAP_POST_MARKER != status->state)
    {
        button_app_last_post_distance_step = 0u;
        return;
    }

    post_distance_step =
        (uint32)(status->brake_distance_m * 100.0f);
    if (button_app_last_post_distance_step != post_distance_step)
    {
        button_app_last_post_distance_step = post_distance_step;
        button_app_force_render = 1u;
    }
}

static void button_app_check_capture_status(void)
{
#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
    uint8 capture_ready;

    if (BUTTON_APP_VIEW_CAPTURE != button_app_view)
    {
        button_app_last_capture_ready = 2u;
        return;
    }
    capture_ready = drive_balance_demo_app_capture_ready();
    if (button_app_last_capture_ready != capture_ready)
    {
        button_app_last_capture_ready = capture_ready;
        button_app_force_render = 1u;
    }
#endif
}

static void button_app_render_no_load_result(void)
{
    const no_load_lap_status_t *status = no_load_lap_app_get_status();
    uint32 distance_mm = (uint32)(status->distance_m * 1000.0f);
    const char *title;

    if (NO_LOAD_LAP_COMPLETE == status->state)
    {
        title = "LAP COMPLETE";
    }
    else if (NO_LOAD_LAP_TIMEOUT == status->state)
    {
        title = "LAP TIMEOUT";
    }
    else if (NO_LOAD_LAP_LINE_LOST == status->state)
    {
        title = "LINE LOST";
    }
    else if (NO_LOAD_LAP_SENSOR_OFFLINE == status->state)
    {
        title = "SENSOR OFFLINE";
    }
    else
    {
        title = "LAP STOPPED";
    }

    oled_clear();
    oled_show_string(0u, 0u, title, OLED_FONT_6X8);
    oled_show_string(0u, 2u, "TIME:", OLED_FONT_6X8);
    oled_show_uint(30u, 2u, status->elapsed_ms, OLED_FONT_6X8);
    oled_show_string(78u, 2u, "ms", OLED_FONT_6X8);
    oled_show_string(0u, 4u, "DIST:", OLED_FONT_6X8);
    oled_show_uint(30u, 4u, distance_mm, OLED_FONT_6X8);
    oled_show_string(78u, 4u, "mm", OLED_FONT_6X8);
    oled_show_string(0u, 7u, "SW4 BACK", OLED_FONT_6X8);
}

static void button_app_render_ab_result(void)
{
    const ab_run_status_t *status = ab_run_app_get_status();
    const char *title = (AB_RUN_COMPLETE == status->state) ?
        ((0u != status->error_requirement_met) ? "AB COMPLETE" :
                                                "AB ERROR >1CM") :
        "AB STOPPED";

    oled_clear();
    oled_show_string(0u, 0u, title, OLED_FONT_6X8);
    oled_show_string(0u, 2u, "TIME:", OLED_FONT_6X8);
    oled_show_uint(30u, 2u, status->elapsed_ms, OLED_FONT_6X8);
    oled_show_string(78u, 2u, "ms", OLED_FONT_6X8);
    oled_show_string(0u, 4u, "MAX ERR:", OLED_FONT_6X8);
    oled_show_uint(48u, 4u,
        (uint32)(status->max_abs_error_m * 1000.0f), OLED_FONT_6X8);
    oled_show_string(78u, 4u, "mm", OLED_FONT_6X8);
    oled_show_string(0u, 7u, "SW4 BACK", OLED_FONT_6X8);
}

static void button_app_render_stop_test_result(void)
{
    const stop_test_status_t *status = stop_test_app_get_status();
    const char *title;

    if (STOP_TEST_COMPLETE == status->state)
    {
        title = (0u != status->overall_requirement_met) ?
            "STOP TEST OK" : "STOP TEST FAIL";
    }
    else if (STOP_TEST_TIMEOUT == status->state)
    {
        title = "STOP TIMEOUT";
    }
    else if (STOP_TEST_FAULT == status->state)
    {
        title = "STOP FAULT";
    }
    else
    {
        title = "STOP TEST STOP";
    }

    oled_clear();
    oled_show_string(0u, 0u, title, OLED_FONT_6X8);
    oled_show_string(0u, 2u, "TIME:", OLED_FONT_6X8);
    oled_show_uint(30u, 2u, status->elapsed_ms, OLED_FONT_6X8);
    oled_show_string(78u, 2u, "ms", OLED_FONT_6X8);
    oled_show_string(0u, 4u, "+ERR:", OLED_FONT_6X8);
    oled_show_uint(36u, 4u,
        (uint32)(status->positive_max_abs_error_m * 1000.0f + 0.5f),
        OLED_FONT_6X8);
    oled_show_string(66u, 4u, "mm", OLED_FONT_6X8);
    oled_show_string(0u, 5u, "-ERR:", OLED_FONT_6X8);
    oled_show_uint(36u, 5u,
        (uint32)(status->negative_max_abs_error_m * 1000.0f + 0.5f),
        OLED_FONT_6X8);
    oled_show_string(66u, 5u, "mm", OLED_FONT_6X8);
    oled_show_string(0u, 7u, "SW4 BACK", OLED_FONT_6X8);
}

#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
static void button_app_render_drive_result(void)
{
    const drive_balance_demo_status_t *status =
        drive_balance_demo_app_get_status();
    const char *title;

    if (DRIVE_BALANCE_DEMO_COMPLETE == status->state)
    {
        title = (0u != status->error_requirement_met) ?
            "BALL LAP OK" : "BALL ERR >1CM";
    }
    else if (DRIVE_BALANCE_DEMO_TIMEOUT == status->state)
    {
        title = "BALL LAP TIMEOUT";
    }
    else
    {
        title = "BALL LAP STOP";
    }

    oled_clear();
    oled_show_string(0u, 0u, title, OLED_FONT_6X8);
    oled_show_string(0u, 2u, "TIME:", OLED_FONT_6X8);
    oled_show_uint(30u, 2u, status->elapsed_ms, OLED_FONT_6X8);
    oled_show_string(78u, 2u, "ms", OLED_FONT_6X8);
    oled_show_string(0u, 4u, "MAX ERR:", OLED_FONT_6X8);
    oled_show_uint(48u, 4u,
        (uint32)(status->max_abs_error_m * 1000.0f), OLED_FONT_6X8);
    oled_show_string(78u, 4u, "mm", OLED_FONT_6X8);
    oled_show_string(0u, 7u, "SW4 BACK", OLED_FONT_6X8);
}
#endif

static void button_app_render(void)
{
    if (BUTTON_APP_VIEW_MENU == button_app_view)
    {
        button_app_render_menu();
    }
    else if (BUTTON_APP_VIEW_CONFIRM == button_app_view)
    {
        button_app_render_dialog("CONFIRM MODE", "Run this mode?");
    }
    else if (BUTTON_APP_VIEW_CAPTURE == button_app_view)
    {
#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
        button_app_render_capture();
#endif
    }
    else if (BUTTON_APP_VIEW_RUNNING == button_app_view)
    {
        button_app_render_running();
    }
    else if (BUTTON_APP_VIEW_REJECTED == button_app_view)
    {
        button_app_render_dialog("MODE NOT READY", "Check system state");
    }
    else
    {
        if (BUTTON_APP_MODE_STOP_TEST == button_app_running_mode)
            button_app_render_stop_test_result();
        else if (BUTTON_APP_MODE_AB == button_app_running_mode)
            button_app_render_ab_result();
#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
        else if ((BUTTON_APP_MODE_BALL_LAP == button_app_running_mode) ||
                 (BUTTON_APP_MODE_ARBITRARY == button_app_running_mode))
            button_app_render_drive_result();
#endif
        else
            button_app_render_no_load_result();
    }
    oled_refresh();
}

static uint8 button_app_start_selected_mode(void)
{
    switch (button_app_selected_mode)
    {
        case BUTTON_APP_MODE_NO_LOAD:
#if (BALANCE_CONTROL_ENABLE != 0u)
            balance_app_cancel_motion();
#endif
            return no_load_lap_app_start();

        case BUTTON_APP_MODE_STOP_TEST:
            return stop_test_app_start();

        case BUTTON_APP_MODE_BALL_LAP:
#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
            return drive_balance_demo_app_start_center();
#else
            return 0u;
#endif

        case BUTTON_APP_MODE_ARBITRARY:
#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
            return drive_balance_demo_app_prepare_captured();
#else
            return 0u;
#endif

        case BUTTON_APP_MODE_AB:
            return ab_run_app_start();

        default:
            return 0u;
    }
}

static void button_app_stop_running_mode(void)
{
    if (BUTTON_APP_MODE_NO_LOAD == button_app_running_mode)
    {
        no_load_lap_app_stop();
        return;
    }
    if (BUTTON_APP_MODE_AB == button_app_running_mode)
    {
        ab_run_app_stop();
        return;
    }
    if (BUTTON_APP_MODE_STOP_TEST == button_app_running_mode)
    {
        stop_test_app_stop();
        return;
    }
#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
    if ((BUTTON_APP_MODE_BALL_LAP == button_app_running_mode) ||
        (BUTTON_APP_MODE_ARBITRARY == button_app_running_mode))
    {
        drive_balance_demo_app_stop();
        return;
    }
#endif
    motor_app_stop();
#if (BALANCE_CONTROL_ENABLE != 0u)
    balance_app_cancel_motion();
#endif
}

static void button_app_handle_press(button_id_t pressed)
{
    uint8 view_changed = 0u;

    if (BUTTON_APP_VIEW_MENU == button_app_view)
    {
        if (BUTTON_ID_SW1 == pressed)
        {
            button_app_selected_mode = (button_app_mode_enum)(
                ((uint8)button_app_selected_mode + 1u) %
                BUTTON_APP_MODE_COUNT);
            view_changed = 1u;
        }
        else if (BUTTON_ID_SW2 == pressed)
        {
            button_app_selected_mode = (button_app_mode_enum)(
                ((uint8)button_app_selected_mode +
                 BUTTON_APP_MODE_COUNT - 1u) % BUTTON_APP_MODE_COUNT);
            view_changed = 1u;
        }
        else if (BUTTON_ID_SW3 == pressed)
        {
            button_app_view = BUTTON_APP_VIEW_CONFIRM;
            view_changed = 1u;
        }
    }
    else if ((BUTTON_APP_VIEW_CONFIRM == button_app_view) ||
             (BUTTON_APP_VIEW_REJECTED == button_app_view))
    {
        if (BUTTON_ID_SW3 == pressed)
        {
            if (0u != button_app_start_selected_mode())
            {
                button_app_running_mode = button_app_selected_mode;
                if (BUTTON_APP_MODE_ARBITRARY ==
                    button_app_selected_mode)
                {
                    button_app_view = BUTTON_APP_VIEW_CAPTURE;
                    heartbeat_hw_uart_send_string(
                        "[mode] mode 5 waiting capture\r\n");
                }
                else
                {
                    button_app_view = BUTTON_APP_VIEW_RUNNING;
                    heartbeat_hw_uart_send_string("[mode] started\r\n");
                }
            }
            else
            {
                button_app_view = BUTTON_APP_VIEW_REJECTED;
                heartbeat_hw_uart_send_string(
                    "[mode] start rejected\r\n");
            }
            view_changed = 1u;
        }
        else if (BUTTON_ID_SW4 == pressed)
        {
            button_app_view = BUTTON_APP_VIEW_MENU;
            view_changed = 1u;
        }
    }
    else if (BUTTON_APP_VIEW_CAPTURE == button_app_view)
    {
#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
        if (BUTTON_ID_SW3 == pressed)
        {
            if (0u != drive_balance_demo_app_start_captured())
            {
                button_app_view = BUTTON_APP_VIEW_RUNNING;
                heartbeat_hw_uart_send_string(
                    "[mode] mode 5 target captured; started\r\n");
            }
            else
            {
                heartbeat_hw_uart_send_string(
                    "[mode] mode 5 capture not ready\r\n");
            }
            view_changed = 1u;
        }
        else if (BUTTON_ID_SW4 == pressed)
        {
            drive_balance_demo_app_stop();
            button_app_view = BUTTON_APP_VIEW_MENU;
            heartbeat_hw_uart_send_string(
                "[mode] mode 5 capture canceled\r\n");
            view_changed = 1u;
        }
#endif
    }
    else if ((BUTTON_APP_VIEW_RUNNING == button_app_view) &&
             (BUTTON_ID_SW4 == pressed))
    {
        button_app_stop_running_mode();
        button_app_view = BUTTON_APP_VIEW_MENU;
        heartbeat_hw_uart_send_string("[mode] stopped\r\n");
        view_changed = 1u;
    }
    else if ((BUTTON_APP_VIEW_RESULT == button_app_view) &&
             (BUTTON_ID_SW4 == pressed))
    {
        if (BUTTON_APP_MODE_NO_LOAD == button_app_running_mode)
        {
            motor_app_stop();
        }
        if (BUTTON_APP_MODE_STOP_TEST == button_app_running_mode)
        {
            stop_test_app_stop();
        }
        button_app_view = BUTTON_APP_VIEW_MENU;
        view_changed = 1u;
    }

    if (0u != view_changed)
    {
        button_app_force_render = 1u;
    }
}

static void button_app_check_mode_completion(void)
{
    if ((BUTTON_APP_VIEW_RUNNING == button_app_view) &&
        (BUTTON_APP_MODE_STOP_TEST == button_app_running_mode) &&
        (0u == stop_test_app_is_running()))
    {
        button_app_view = BUTTON_APP_VIEW_RESULT;
        button_app_force_render = 1u;
        heartbeat_hw_uart_send_string("[mode] stop test ended\r\n");
        return;
    }
    if ((BUTTON_APP_VIEW_RUNNING == button_app_view) &&
        (BUTTON_APP_MODE_AB == button_app_running_mode) &&
        (0u == ab_run_app_is_running()))
    {
        button_app_view = BUTTON_APP_VIEW_RESULT;
        button_app_force_render = 1u;
        heartbeat_hw_uart_send_string("[mode] AB ended\r\n");
        return;
    }
    if ((BUTTON_APP_VIEW_RUNNING == button_app_view) &&
        (BUTTON_APP_MODE_NO_LOAD == button_app_running_mode) &&
        (0u == no_load_lap_app_is_running()))
    {
        button_app_view = BUTTON_APP_VIEW_RESULT;
        button_app_force_render = 1u;
        heartbeat_hw_uart_send_string("[mode] lap ended\r\n");
        return;
    }
#if (BALANCE_DRIVE_DEMO_ENABLE != 0u)
    if ((BUTTON_APP_VIEW_RUNNING == button_app_view) &&
        ((BUTTON_APP_MODE_BALL_LAP == button_app_running_mode) ||
         (BUTTON_APP_MODE_ARBITRARY == button_app_running_mode)) &&
        (0u == drive_balance_demo_app_is_running()))
    {
        button_app_view = BUTTON_APP_VIEW_RESULT;
        button_app_force_render = 1u;
        heartbeat_hw_uart_send_string("[mode] completed\r\n");
    }
#endif
}

void button_app_init(void)
{
    button_init();
    button_app_previous = BUTTON_ID_NONE;
    button_app_selected_mode = BUTTON_APP_MODE_NO_LOAD;
    button_app_running_mode = BUTTON_APP_MODE_NO_LOAD;
    button_app_view = BUTTON_APP_VIEW_MENU;
    button_app_force_render = 1u;
    button_app_last_vision_online = 2u;
    button_app_last_no_load_state = NO_LOAD_LAP_IDLE;
    button_app_last_post_distance_step = 0u;
    button_app_last_capture_ready = 2u;
    oled_app_set_dashboard_enabled(0u);
}

void button_app_process(void)
{
    button_id_t active;

    button_process();
    active = button_get_active();
    if ((BUTTON_ID_NONE != active) && (active != button_app_previous))
    {
        button_app_handle_press(active);
    }
    button_app_previous = active;
    button_app_check_mode_completion();
    button_app_check_no_load_status();
    button_app_check_capture_status();
    button_app_check_vision_status();

    if (0u == oled_is_ready())
    {
        button_app_force_render = 1u;
        return;
    }
    if (0u == button_app_force_render)
    {
        return;
    }
    button_app_force_render = 0u;
    button_app_render();
}

button_app_mode_enum button_app_get_selected_mode(void)
{
    return button_app_selected_mode;
}

uint8 button_app_is_running(void)
{
    return (BUTTON_APP_VIEW_RUNNING == button_app_view) ? 1u : 0u;
}
