#include "emm42_demo_app.h"

#include "button.h"
#include "balance_linkage.h"
#include "control_config.h"
#include "emm42.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "vision_link.h"

#define EMM42_DEMO_ADDRESS                  (EMM42_DEFAULT_ADDRESS)
#define EMM42_DEMO_RPM                      BALANCE_EMM42_MOVE_RPM
#define EMM42_DEMO_LEVEL_RPM                BALANCE_LEVEL_RETURN_RPM
#define EMM42_DEMO_ACCELERATION             BALANCE_EMM42_ACCELERATION
#define EMM42_DEMO_POWER_WAIT_MS            (3000u)
#define EMM42_DEMO_COMMAND_WAIT_MS          (100u)
#define EMM42_DEMO_ANGLE_READY_MS           (1000u)
#define EMM42_DEMO_RECORD_MS                (4000u)
#define EMM42_DEMO_POSITION_QUERY_PERIOD_MS (20u)
#define EMM42_DEMO_TARGET_COUNT             (4u)
#define EMM42_DEMO_MIN_VISION_CONFIDENCE    (30u)

static const float emm42_demo_target_angles_deg[EMM42_DEMO_TARGET_COUNT] =
{
    0.0f, -2.0f, -3.0f, -4.0f,
};

static emm42_demo_state_enum emm42_demo_state;
static emm42_frame_t emm42_demo_rx_frame;
static uint32 emm42_demo_state_start_ms;
static uint32 emm42_demo_last_query_ms;
static uint16 emm42_demo_trial_id;
static uint8 emm42_demo_target_index;
static button_id_t emm42_demo_previous_button;
static float emm42_demo_target_lever_deg;
static float emm42_demo_target_motor_deg;
static float emm42_demo_motor_feedback_deg;
static uint8 emm42_demo_motor_feedback_valid;

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

static uint8 emm42_demo_move_to_current_angle(uint32 now_ms)
{
    uint16 move_rpm = (0u == emm42_demo_target_index) ?
        EMM42_DEMO_LEVEL_RPM : EMM42_DEMO_RPM;

    emm42_demo_target_lever_deg =
        emm42_demo_target_angles_deg[emm42_demo_target_index];
    if (0u == balance_linkage_motor_from_physical_lever_deg(
            emm42_demo_target_lever_deg, &emm42_demo_target_motor_deg))
    {
        return 0u;
    }
    if (0u == emm42_move_angle(EMM42_DEMO_ADDRESS,
                               emm42_demo_target_motor_deg,
                               move_rpm, EMM42_DEMO_ACCELERATION,
                               EMM42_POSITION_ABSOLUTE, 0u))
    {
        return 0u;
    }
    emm42_demo_motor_feedback_valid = 0u;
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
    if (((EMM42_DEMO_WAIT_ANGLE == emm42_demo_state) ||
         (EMM42_DEMO_READY == emm42_demo_state) ||
         (EMM42_DEMO_RECORDING == emm42_demo_state)) &&
        ((now_ms - emm42_demo_last_query_ms) >=
         EMM42_DEMO_POSITION_QUERY_PERIOD_MS))
    {
        emm42_demo_last_query_ms = now_ms;
        (void)emm42_query_position(EMM42_DEMO_ADDRESS);
    }
}

static button_id_t emm42_demo_take_button_edge(void)
{
    button_id_t active = button_get_active();
    button_id_t edge = BUTTON_ID_NONE;

    if ((BUTTON_ID_NONE != active) &&
        (active != emm42_demo_previous_button))
    {
        edge = active;
    }
    emm42_demo_previous_button = active;
    return edge;
}

static uint8 emm42_demo_ball_measurement_ready(void)
{
    vision_link_snapshot_t measurement;
    uint8 required = VISION_LINK_FLAG_MEASURED_VALID |
                     VISION_LINK_FLAG_TRACKER_READY |
                     VISION_LINK_FLAG_CALIBRATION_VALID;

    if (0u == vision_link_get_valid_measurement(&measurement))
    {
        return 0u;
    }
    return (((measurement.flags & required) == required) &&
            (measurement.confidence >= EMM42_DEMO_MIN_VISION_CONFIDENCE)) ?
        1u : 0u;
}

void emm42_demo_app_init(void)
{
    uint32 now_ms = heartbeat_get_ms();

    emm42_demo_state_start_ms = now_ms;
    emm42_demo_last_query_ms = now_ms;
    emm42_demo_state = EMM42_DEMO_WAIT_POWER;
    emm42_demo_trial_id = 0u;
    emm42_demo_target_index = 0u;
    emm42_demo_previous_button = BUTTON_ID_NONE;
    emm42_demo_target_lever_deg = emm42_demo_target_angles_deg[0];
    emm42_demo_target_motor_deg = 0.0f;
    emm42_demo_motor_feedback_deg = 0.0f;
    emm42_demo_motor_feedback_valid = 0u;
    emm42_demo_rx_frame.length = 0u;

    emm42_init();
    heartbeat_hw_uart_send_string(
        "[dynamics-cal] lever must rest at lower mechanical stop\r\n");
}

void emm42_demo_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();
    button_id_t button_edge;

    emm42_demo_drain_position();
    button_edge = emm42_demo_take_button_edge();
    if ((BUTTON_ID_SW4 == button_edge) &&
        (EMM42_DEMO_ERROR != emm42_demo_state))
    {
        emm42_demo_fail(now_ms, "[dynamics-cal] aborted by SW4\r\n");
    }

    switch (emm42_demo_state)
    {
        case EMM42_DEMO_WAIT_POWER:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_POWER_WAIT_MS))
            {
                if (0u == emm42_set_current_position_zero(EMM42_DEMO_ADDRESS))
                {
                    emm42_demo_fail(now_ms,
                        "[dynamics-cal] zero command failed\r\n");
                    break;
                }
                heartbeat_hw_uart_send_string(
                    "[dynamics-cal] lower stop set as motor zero\r\n");
                emm42_demo_set_state(EMM42_DEMO_WAIT_ZERO, now_ms);
            }
            break;

        case EMM42_DEMO_WAIT_ZERO:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_COMMAND_WAIT_MS))
            {
                if (0u == emm42_set_enabled(EMM42_DEMO_ADDRESS, 1u, 0u))
                {
                    emm42_demo_fail(now_ms,
                        "[dynamics-cal] enable command failed\r\n");
                    break;
                }
                heartbeat_hw_uart_send_string("[dynamics-cal] enabled\r\n");
                emm42_demo_set_state(EMM42_DEMO_WAIT_ENABLE, now_ms);
            }
            break;

        case EMM42_DEMO_WAIT_ENABLE:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_COMMAND_WAIT_MS))
            {
                emm42_demo_set_state(EMM42_DEMO_MOVE_ANGLE, now_ms);
            }
            break;

        case EMM42_DEMO_MOVE_ANGLE:
            if (0u == emm42_demo_move_to_current_angle(now_ms))
            {
                emm42_demo_fail(now_ms,
                    "[dynamics-cal] angle command failed\r\n");
                break;
            }
            heartbeat_hw_uart_send_string(
                "[dynamics-cal] moving to test angle\r\n");
            emm42_demo_set_state(EMM42_DEMO_WAIT_ANGLE, now_ms);
            break;

        case EMM42_DEMO_WAIT_ANGLE:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_ANGLE_READY_MS))
            {
                heartbeat_hw_uart_send_string(
                    "[dynamics-cal] ready: SW1 record, SW2 next\r\n");
                emm42_demo_set_state(EMM42_DEMO_READY, now_ms);
            }
            break;

        case EMM42_DEMO_READY:
            if (BUTTON_ID_SW1 == button_edge)
            {
                if (0u == emm42_demo_ball_measurement_ready())
                {
                    heartbeat_hw_uart_send_string(
                        "[dynamics-cal] SW1 rejected: ball not detected\r\n");
                }
                else
                {
                    emm42_demo_trial_id++;
                    heartbeat_hw_uart_send_string(
                        "[dynamics-cal] recording trial\r\n");
                    emm42_demo_set_state(EMM42_DEMO_RECORDING, now_ms);
                }
            }
            else if (BUTTON_ID_SW2 == button_edge)
            {
                if ((emm42_demo_target_index + 1u) < EMM42_DEMO_TARGET_COUNT)
                {
                    emm42_demo_target_index++;
                    emm42_demo_set_state(EMM42_DEMO_MOVE_ANGLE, now_ms);
                }
                else
                {
                    heartbeat_hw_uart_send_string(
                        "[dynamics-cal] all angles complete; holding\r\n");
                    emm42_demo_set_state(EMM42_DEMO_DONE, now_ms);
                }
            }
            break;

        case EMM42_DEMO_RECORDING:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_RECORD_MS))
            {
                heartbeat_hw_uart_send_string(
                    "[dynamics-cal] trial complete; reset ball\r\n");
                emm42_demo_set_state(EMM42_DEMO_READY, now_ms);
            }
            break;

        case EMM42_DEMO_DONE:
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

uint16 emm42_demo_app_get_trial_id(void)
{
    return emm42_demo_trial_id;
}

float emm42_demo_app_get_target_lever_deg(void)
{
    return emm42_demo_target_lever_deg;
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

uint8 emm42_demo_app_is_recording(void)
{
    return (EMM42_DEMO_RECORDING == emm42_demo_state) ? 1u : 0u;
}

uint8 emm42_demo_app_is_active(void)
{
    return (EMM42_DEMO_ERROR != emm42_demo_state) ? 1u : 0u;
}
