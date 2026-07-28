#include "grayscale.h"

#include "grayscale_hw.h"
#include "zf_driver_timer.h"

typedef enum
{
    GRAYSCALE_STATE_SELECT = 0,
    GRAYSCALE_STATE_WAIT_SETTLE,
    GRAYSCALE_STATE_READ,
} grayscale_state_enum;

#define GRAYSCALE_SETTLE_TIMER         (TIM_G7)

static uint8 grayscale_values[GRAYSCALE_CHANNELS];
static uint8 grayscale_scan_ready;
static uint8 grayscale_channel;
static grayscale_state_enum grayscale_state;
static uint16 grayscale_settle_start_us;

static uint8 grayscale_settle_elapsed(uint16 start_us)
{
    uint16 elapsed_us = (uint16)(timer_get(GRAYSCALE_SETTLE_TIMER) - start_us);

    return (elapsed_us >= (uint16)GRAYSCALE_HW_SETTLE_US) ? 1u : 0u;
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
    }

    grayscale_scan_ready = 0;
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
            grayscale_state = GRAYSCALE_STATE_WAIT_SETTLE;
            break;

        case GRAYSCALE_STATE_WAIT_SETTLE:
            if (!grayscale_settle_elapsed(grayscale_settle_start_us))
            {
                return;
            }
            grayscale_state = GRAYSCALE_STATE_READ;
            break;

        case GRAYSCALE_STATE_READ:
            grayscale_values[grayscale_channel] = grayscale_hw_read_out();

            if ((GRAYSCALE_CHANNELS - 1u) <= grayscale_channel)
            {
                grayscale_scan_ready = 1;
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
