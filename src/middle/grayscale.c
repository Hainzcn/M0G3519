#include "grayscale.h"

#include "ti_msp_dl_config.h"

typedef enum
{
    GRAYSCALE_STATE_SELECT = 0,
    GRAYSCALE_STATE_WAIT_SETTLE,
    GRAYSCALE_STATE_READ,
} grayscale_state_enum;

static uint8 grayscale_values[GRAYSCALE_CHANNELS];
static uint8 grayscale_scan_ready;
static uint8 grayscale_channel;
static grayscale_state_enum grayscale_state;
static uint32 grayscale_settle_start_val;

static uint32 grayscale_systick_ticks_elapsed(uint32 start_val, uint32 now_val)
{
    uint32 load = SysTick->LOAD;

    if (now_val <= start_val)
    {
        return start_val - now_val;
    }

    return start_val + (load - now_val);
}

static uint8 grayscale_settle_elapsed(uint32 start_val)
{
    uint32 elapsed_ticks = grayscale_systick_ticks_elapsed(start_val, SysTick->VAL);
    uint32 required_ticks = (uint32)GRAYSCALE_HW_SETTLE_US * (CPUCLK_FREQ / 1000000u);

    return (elapsed_ticks >= required_ticks) ? 1u : 0u;
}

void grayscale_init(void)
{
    uint8 i;

    grayscale_hw_init();

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
            grayscale_settle_start_val = SysTick->VAL;
            grayscale_state = GRAYSCALE_STATE_WAIT_SETTLE;
            break;

        case GRAYSCALE_STATE_WAIT_SETTLE:
            if (!grayscale_settle_elapsed(grayscale_settle_start_val))
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
