#include "emm42_demo_app.h"

#include "balance_linkage.h"
#include "control_config.h"
#include "emm42.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"

#define EMM42_DEMO_ADDRESS                  (EMM42_DEFAULT_ADDRESS)
#define EMM42_DEMO_RPM                      (30u)
#define EMM42_DEMO_ACCELERATION             (20u)
#define EMM42_DEMO_POWER_WAIT_MS            (3000u)
#define EMM42_DEMO_COMMAND_WAIT_MS          (100u)
#define EMM42_DEMO_LEVEL_TIMEOUT_MS         (3000u)
#define EMM42_DEMO_LEVEL_SETTLE_MS          (200u)
#define EMM42_DEMO_LEVEL_TOLERANCE_DEG      (1.0f)
#define EMM42_DEMO_ENDPOINT_WAIT_MS         (1500u)
#define EMM42_DEMO_POSITION_QUERY_PERIOD_MS (10u)
#define EMM42_DEMO_ALPHA_DEG                (5.0f)

static emm42_demo_state_enum emm42_demo_state;
static emm42_frame_t emm42_demo_rx_frame;
static uint32 emm42_demo_state_start_ms;
static uint32 emm42_demo_last_query_ms;
static uint32 emm42_demo_level_tolerance_start_ms;
static float emm42_demo_level_motor_deg;
static float emm42_demo_positive_motor_deg;
static float emm42_demo_negative_motor_deg;
static float emm42_demo_target_motor_deg;
static float emm42_demo_target_angle_deg;
static float emm42_demo_motor_feedback_deg;
static uint8 emm42_demo_motor_feedback_valid;
static uint8 emm42_demo_level_tolerance_active;

static float emm42_demo_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8 emm42_demo_elapsed(uint32 now_ms, uint32 wait_ms)
{
    return ((now_ms - emm42_demo_state_start_ms) >= wait_ms) ? 1u : 0u;
}

static void emm42_demo_set_state(emm42_demo_state_enum state, uint32 now_ms)
{
    emm42_demo_state = state;
    emm42_demo_state_start_ms = now_ms;
}

static void emm42_demo_fail(uint32 now_ms, const char *message)
{
    (void)emm42_stop(EMM42_DEMO_ADDRESS, 0u);
    heartbeat_hw_uart_send_string(message);
    emm42_demo_set_state(EMM42_DEMO_ERROR, now_ms);
}

static uint8 emm42_demo_calculate_targets(void)
{
    return (
        (0u != balance_linkage_relative_motor_deg(
            BALANCE_STARTUP_LEVER_ANGLE_DEG, 0.0f,
            &emm42_demo_level_motor_deg)) &&
        (0u != balance_linkage_relative_motor_deg(
            BALANCE_STARTUP_LEVER_ANGLE_DEG, EMM42_DEMO_ALPHA_DEG,
            &emm42_demo_positive_motor_deg)) &&
        (0u != balance_linkage_relative_motor_deg(
            BALANCE_STARTUP_LEVER_ANGLE_DEG, -EMM42_DEMO_ALPHA_DEG,
            &emm42_demo_negative_motor_deg))) ? 1u : 0u;
}

static uint8 emm42_demo_move_to(float lever_angle_deg, float motor_angle_deg,
                                uint32 now_ms)
{
    if (0u == emm42_move_angle(EMM42_DEMO_ADDRESS, motor_angle_deg,
                               EMM42_DEMO_RPM, EMM42_DEMO_ACCELERATION,
                               EMM42_POSITION_ABSOLUTE, 0u))
    {
        return 0u;
    }
    emm42_demo_target_angle_deg = lever_angle_deg;
    emm42_demo_target_motor_deg = motor_angle_deg;
    emm42_demo_last_query_ms = now_ms;
    return 1u;
}

static void emm42_demo_drain_position(void)
{
    float position_deg;

    while (0u != emm42_read_frame(&emm42_demo_rx_frame))
    {
        if (0u != emm42_decode_position_deg(
                &emm42_demo_rx_frame, EMM42_DEMO_ADDRESS, &position_deg))
        {
            emm42_demo_motor_feedback_deg = position_deg;
            emm42_demo_motor_feedback_valid = 1u;
        }
    }
}

static void emm42_demo_query_position_if_due(uint32 now_ms)
{
    if ((EMM42_DEMO_ERROR != emm42_demo_state) &&
        ((now_ms - emm42_demo_last_query_ms) >=
         EMM42_DEMO_POSITION_QUERY_PERIOD_MS))
    {
        emm42_demo_last_query_ms = now_ms;
        (void)emm42_query_position(EMM42_DEMO_ADDRESS);
    }
}

void emm42_demo_app_init(void)
{
    uint32 now_ms = heartbeat_get_ms();

    emm42_demo_state_start_ms = now_ms;
    emm42_demo_last_query_ms = now_ms;
    emm42_demo_level_tolerance_start_ms = now_ms;
    emm42_demo_state = EMM42_DEMO_WAIT_POWER;
    emm42_demo_target_angle_deg = BALANCE_STARTUP_LEVER_ANGLE_DEG;
    emm42_demo_target_motor_deg = 0.0f;
    emm42_demo_motor_feedback_deg = 0.0f;
    emm42_demo_motor_feedback_valid = 0u;
    emm42_demo_level_tolerance_active = 0u;
    emm42_demo_rx_frame.length = 0u;

    if (0u == emm42_demo_calculate_targets())
    {
        heartbeat_hw_uart_send_string(
            "[balance-demo] linkage target unreachable\r\n");
        emm42_demo_state = EMM42_DEMO_ERROR;
        return;
    }

    emm42_init();
    heartbeat_hw_uart_send_string(
        "[balance-demo] lever must rest at lower mechanical stop\r\n");
}

void emm42_demo_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();

    emm42_demo_drain_position();
    switch (emm42_demo_state)
    {
        case EMM42_DEMO_WAIT_POWER:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_POWER_WAIT_MS))
            {
                if (0u == emm42_set_current_position_zero(EMM42_DEMO_ADDRESS))
                {
                    emm42_demo_fail(now_ms,
                        "[balance-demo] zero command failed\r\n");
                    break;
                }
                emm42_demo_motor_feedback_valid = 0u;
                emm42_demo_last_query_ms = now_ms;
                heartbeat_hw_uart_send_string(
                    "[balance-demo] lower stop set as motor zero\r\n");
                emm42_demo_set_state(EMM42_DEMO_WAIT_ZERO, now_ms);
            }
            break;

        case EMM42_DEMO_WAIT_ZERO:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_COMMAND_WAIT_MS))
            {
                if (0u == emm42_set_enabled(EMM42_DEMO_ADDRESS, 1u, 0u))
                {
                    emm42_demo_fail(now_ms,
                        "[balance-demo] enable command failed\r\n");
                    break;
                }
                emm42_demo_last_query_ms = now_ms;
                heartbeat_hw_uart_send_string("[balance-demo] enabled\r\n");
                emm42_demo_set_state(EMM42_DEMO_WAIT_ENABLE, now_ms);
            }
            break;

        case EMM42_DEMO_WAIT_ENABLE:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_COMMAND_WAIT_MS))
            {
                emm42_demo_set_state(EMM42_DEMO_MOVE_LEVEL, now_ms);
            }
            break;

        case EMM42_DEMO_MOVE_LEVEL:
            if (0u == emm42_demo_move_to(0.0f, emm42_demo_level_motor_deg,
                                         now_ms))
            {
                emm42_demo_fail(now_ms,
                    "[balance-demo] level command failed\r\n");
                break;
            }
            emm42_demo_level_tolerance_active = 0u;
            heartbeat_hw_uart_send_string(
                "[balance-demo] lower stop -> level\r\n");
            emm42_demo_set_state(EMM42_DEMO_WAIT_LEVEL, now_ms);
            break;

        case EMM42_DEMO_WAIT_LEVEL:
            if (emm42_demo_elapsed(now_ms, EMM42_DEMO_LEVEL_TIMEOUT_MS))
            {
                emm42_demo_fail(now_ms,
                    "[balance-demo] level move timeout\r\n");
                break;
            }
            if ((0u != emm42_demo_motor_feedback_valid) &&
                (emm42_demo_abs(emm42_demo_motor_feedback_deg -
                                emm42_demo_level_motor_deg) <=
                 EMM42_DEMO_LEVEL_TOLERANCE_DEG))
            {
                if (0u == emm42_demo_level_tolerance_active)
                {
                    emm42_demo_level_tolerance_active = 1u;
                    emm42_demo_level_tolerance_start_ms = now_ms;
                }
                else if ((now_ms - emm42_demo_level_tolerance_start_ms) >=
                         EMM42_DEMO_LEVEL_SETTLE_MS)
                {
                    heartbeat_hw_uart_send_string(
                        "[balance-demo] level confirmed; start +/-5 deg\r\n");
                    emm42_demo_set_state(EMM42_DEMO_MOVE_POSITIVE, now_ms);
                }
            }
            else
            {
                emm42_demo_level_tolerance_active = 0u;
            }
            break;

        case EMM42_DEMO_MOVE_POSITIVE:
            if (0u == emm42_demo_move_to(EMM42_DEMO_ALPHA_DEG,
                                         emm42_demo_positive_motor_deg,
                                         now_ms))
            {
                emm42_demo_fail(now_ms,
                    "[balance-demo] +5 deg command failed\r\n");
                break;
            }
            heartbeat_hw_uart_send_string("[balance-demo] lever -> +5 deg\r\n");
            emm42_demo_set_state(EMM42_DEMO_WAIT_POSITIVE, now_ms);
            break;

        case EMM42_DEMO_WAIT_POSITIVE:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_ENDPOINT_WAIT_MS))
            {
                emm42_demo_set_state(EMM42_DEMO_MOVE_NEGATIVE, now_ms);
            }
            break;

        case EMM42_DEMO_MOVE_NEGATIVE:
            if (0u == emm42_demo_move_to(-EMM42_DEMO_ALPHA_DEG,
                                         emm42_demo_negative_motor_deg,
                                         now_ms))
            {
                emm42_demo_fail(now_ms,
                    "[balance-demo] -5 deg command failed\r\n");
                break;
            }
            heartbeat_hw_uart_send_string("[balance-demo] lever -> -5 deg\r\n");
            emm42_demo_set_state(EMM42_DEMO_WAIT_NEGATIVE, now_ms);
            break;

        case EMM42_DEMO_WAIT_NEGATIVE:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_ENDPOINT_WAIT_MS))
            {
                emm42_demo_set_state(EMM42_DEMO_MOVE_POSITIVE, now_ms);
            }
            break;

        case EMM42_DEMO_ERROR:
        default:
            break;
    }
    emm42_demo_query_position_if_due(now_ms);
}

emm42_demo_state_enum emm42_demo_app_get_state(void)
{
    return emm42_demo_state;
}

float emm42_demo_app_get_target_angle_deg(void)
{
    return emm42_demo_target_angle_deg;
}

float emm42_demo_app_get_target_motor_deg(void)
{
    return emm42_demo_target_motor_deg;
}

float emm42_demo_app_get_motor_feedback_deg(void)
{
    return emm42_demo_motor_feedback_deg;
}

uint8 emm42_demo_app_is_motor_feedback_valid(void)
{
    return emm42_demo_motor_feedback_valid;
}

uint8 emm42_demo_app_is_active(void)
{
    return (EMM42_DEMO_ERROR != emm42_demo_state) ? 1u : 0u;
}
