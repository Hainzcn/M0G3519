#include "emm42_demo_app.h"

#include "emm42.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"

#define EMM42_DEMO_ADDRESS             (EMM42_DEFAULT_ADDRESS)
#define EMM42_DEMO_TURNS               (2u)
#define EMM42_DEMO_PULSES              ((int32)(EMM42_DEFAULT_PULSES_PER_REV * EMM42_DEMO_TURNS))
#define EMM42_DEMO_RPM                 (60u)
#define EMM42_DEMO_ACCELERATION        (20u)
#define EMM42_DEMO_POWER_WAIT_MS       (500u)
#define EMM42_DEMO_COMMAND_WAIT_MS     (100u)
#define EMM42_DEMO_MOTION_WAIT_MS      (3500u)
#define EMM42_DEMO_ZERO_WAIT_MS        (1000u)

static emm42_demo_state_enum emm42_demo_state;
static uint32 emm42_demo_state_start_ms;

static uint8 emm42_demo_elapsed(uint32 now_ms, uint32 wait_ms)
{
    return ((now_ms - emm42_demo_state_start_ms) >= wait_ms) ? 1u : 0u;
}

static void emm42_demo_set_state(emm42_demo_state_enum state, uint32 now_ms)
{
    emm42_demo_state = state;
    emm42_demo_state_start_ms = now_ms;
}

static void emm42_demo_fail(uint32 now_ms)
{
    (void)emm42_stop(EMM42_DEMO_ADDRESS, 0u);
    heartbeat_hw_uart_send_string("[emm42-demo] command failed\r\n");
    emm42_demo_set_state(EMM42_DEMO_ERROR, now_ms);
}

void emm42_demo_app_init(void)
{
    emm42_init();
    emm42_demo_state_start_ms = heartbeat_get_ms();
    emm42_demo_state = EMM42_DEMO_WAIT_POWER;
    heartbeat_hw_uart_send_string("[emm42-demo] wait PLC driver power\r\n");
}

void emm42_demo_app_process(void)
{
    uint32 now_ms = heartbeat_get_ms();

    switch (emm42_demo_state)
    {
        case EMM42_DEMO_WAIT_POWER:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_POWER_WAIT_MS))
            {
                if (0u == emm42_set_current_position_zero(EMM42_DEMO_ADDRESS))
                {
                    emm42_demo_fail(now_ms);
                    break;
                }
                heartbeat_hw_uart_send_string("[emm42-demo] origin set\r\n");
                emm42_demo_set_state(EMM42_DEMO_WAIT_ZERO, now_ms);
            }
            break;

        case EMM42_DEMO_WAIT_ZERO:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_COMMAND_WAIT_MS))
            {
                if (0u == emm42_set_enabled(EMM42_DEMO_ADDRESS, 1u, 0u))
                {
                    emm42_demo_fail(now_ms);
                    break;
                }
                heartbeat_hw_uart_send_string("[emm42-demo] enabled\r\n");
                emm42_demo_set_state(EMM42_DEMO_WAIT_ENABLE, now_ms);
            }
            break;

        case EMM42_DEMO_WAIT_ENABLE:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_COMMAND_WAIT_MS))
            {
                emm42_demo_set_state(EMM42_DEMO_FORWARD, now_ms);
            }
            break;

        case EMM42_DEMO_FORWARD:
            if (0u == emm42_move_pulses(EMM42_DEMO_ADDRESS,
                    EMM42_DEMO_PULSES, EMM42_DEMO_RPM,
                    EMM42_DEMO_ACCELERATION,
                    EMM42_POSITION_RELATIVE_TO_CURRENT, 0u))
            {
                emm42_demo_fail(now_ms);
                break;
            }
            heartbeat_hw_uart_send_string("[emm42-demo] forward 2 turns\r\n");
            emm42_demo_set_state(EMM42_DEMO_WAIT_FORWARD, now_ms);
            break;

        case EMM42_DEMO_WAIT_FORWARD:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_MOTION_WAIT_MS))
            {
                emm42_demo_set_state(EMM42_DEMO_REVERSE, now_ms);
            }
            break;

        case EMM42_DEMO_REVERSE:
            if (0u == emm42_move_pulses(EMM42_DEMO_ADDRESS,
                    -EMM42_DEMO_PULSES, EMM42_DEMO_RPM,
                    EMM42_DEMO_ACCELERATION,
                    EMM42_POSITION_RELATIVE_TO_CURRENT, 0u))
            {
                emm42_demo_fail(now_ms);
                break;
            }
            heartbeat_hw_uart_send_string("[emm42-demo] reverse 2 turns\r\n");
            emm42_demo_set_state(EMM42_DEMO_WAIT_REVERSE, now_ms);
            break;

        case EMM42_DEMO_WAIT_REVERSE:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_MOTION_WAIT_MS))
            {
                emm42_demo_set_state(EMM42_DEMO_RETURN_ZERO, now_ms);
            }
            break;

        case EMM42_DEMO_RETURN_ZERO:
            if (0u == emm42_move_pulses(EMM42_DEMO_ADDRESS, 0,
                    EMM42_DEMO_RPM, EMM42_DEMO_ACCELERATION,
                    EMM42_POSITION_ABSOLUTE, 0u))
            {
                emm42_demo_fail(now_ms);
                break;
            }
            heartbeat_hw_uart_send_string("[emm42-demo] absolute return to origin\r\n");
            emm42_demo_set_state(EMM42_DEMO_WAIT_RETURN_ZERO, now_ms);
            break;

        case EMM42_DEMO_WAIT_RETURN_ZERO:
            if (0u != emm42_demo_elapsed(now_ms, EMM42_DEMO_ZERO_WAIT_MS))
            {
                if (0u == emm42_stop(EMM42_DEMO_ADDRESS, 0u))
                {
                    emm42_demo_fail(now_ms);
                    break;
                }
                heartbeat_hw_uart_send_string("[emm42-demo] done at origin\r\n");
                emm42_demo_set_state(EMM42_DEMO_DONE, now_ms);
            }
            break;

        case EMM42_DEMO_DONE:
        case EMM42_DEMO_ERROR:
        default:
            break;
    }
}

emm42_demo_state_enum emm42_demo_app_get_state(void)
{
    return emm42_demo_state;
}
