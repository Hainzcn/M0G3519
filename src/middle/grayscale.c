#include "grayscale.h"

#include "grayscale_hw.h"
#include "heartbeat.h"
#include "zf_driver_timer.h"

typedef enum
{
    GRAYSCALE_STATE_SELECT = 0,
    GRAYSCALE_STATE_WAIT_SETTLE,
    GRAYSCALE_STATE_READ,
} grayscale_state_enum;

#define GRAYSCALE_SETTLE_TIMER         (TIM_G7)

static uint8 grayscale_values[GRAYSCALE_CHANNELS];
static uint8 grayscale_work_values[GRAYSCALE_CHANNELS];
static uint8 grayscale_scan_ready;
static uint32 grayscale_scan_sequence;
static uint8 grayscale_channel;
static grayscale_state_enum grayscale_state;
static uint16 grayscale_settle_start_us;
static uint32 grayscale_settle_start_ms;

static uint8 grayscale_settle_elapsed(uint16 start_us, uint32 start_ms)
{
    uint16 elapsed_us = (uint16)(timer_get(GRAYSCALE_SETTLE_TIMER) - start_us);

    if (elapsed_us >= (uint16)GRAYSCALE_HW_SETTLE_US)
    {
        return 1u;
    }

    /* Never leave the scanner stuck if the dedicated timer stops advancing. */
    return ((heartbeat_get_ms() - start_ms) >= 1u) ? 1u : 0u;
}

void grayscale_init(void)
{
    uint8 i;

    grayscale_hw_init();
    timer_init(GRAYSCALE_SETTLE_TIMER, TIMER_US);
    timer_start(GRAYSCALE_SETTLE_TIMER);

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        grayscale_values[i] = 0;
        grayscale_work_values[i] = 0;
    }

    grayscale_scan_ready = 0;
    grayscale_scan_sequence = 0u;
    grayscale_channel    = 0;
    grayscale_state      = GRAYSCALE_STATE_SELECT;
}

void grayscale_process(void)
{
    switch (grayscale_state)
    {
        case GRAYSCALE_STATE_SELECT:
            grayscale_hw_select_channel(grayscale_channel);
            grayscale_settle_start_us = timer_get(GRAYSCALE_SETTLE_TIMER);
            grayscale_settle_start_ms = heartbeat_get_ms();
            grayscale_state = GRAYSCALE_STATE_WAIT_SETTLE;
            break;

        case GRAYSCALE_STATE_WAIT_SETTLE:
            if (!grayscale_settle_elapsed(grayscale_settle_start_us,
                                          grayscale_settle_start_ms))
            {
                return;
            }
            grayscale_state = GRAYSCALE_STATE_READ;
            break;

        case GRAYSCALE_STATE_READ:
            grayscale_work_values[grayscale_channel] =
                grayscale_hw_read_out();

            if ((GRAYSCALE_CHANNELS - 1u) <= grayscale_channel)
            {
                uint8 i;

                /* Publish only a complete scan so OLED/control never tear. */
                for (i = 0u; i < GRAYSCALE_CHANNELS; i++)
                {
                    grayscale_values[i] = grayscale_work_values[i];
                }
                grayscale_scan_ready = 1;
                grayscale_scan_sequence++;
                grayscale_channel    = 0;
            }
            else
            {
                grayscale_channel ++;
            }

            grayscale_state = GRAYSCALE_STATE_SELECT;
            break;

        default:
            grayscale_state = GRAYSCALE_STATE_SELECT;
            break;
    }
}

const uint8 *grayscale_get_values(void)
{
    return grayscale_values;
}

uint8 grayscale_is_scan_ready(void)
{
    return grayscale_scan_ready;
}

uint8 grayscale_take_scan_ready(void)
{
    if (0u == grayscale_scan_ready)
    {
        return 0u;
    }

    grayscale_scan_ready = 0u;
    return 1u;
}

uint32 grayscale_get_scan_sequence(void)
{
    return grayscale_scan_sequence;
}
