#include "grayscale.h"

#include "grayscale_hw.h"
#include "heartbeat.h"

static uint8 grayscale_values[GRAYSCALE_CHANNELS];
static uint8 grayscale_work_values[GRAYSCALE_CHANNELS];
static uint8 grayscale_scan_ready;
static uint32 grayscale_scan_sequence;
static uint32 grayscale_settle_start_ms;

void grayscale_init(void)
{
    uint8 i;

    grayscale_hw_init();

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        grayscale_values[i] = 0;
        grayscale_work_values[i] = 0;
    }

    grayscale_scan_ready = 0;
    grayscale_scan_sequence = 0u;
    grayscale_settle_start_ms = heartbeat_get_ms() -
        GRAYSCALE_HW_SCAN_PERIOD_MS;
}

void grayscale_process(void)
{
    uint8 i;
    uint32 now_ms = heartbeat_get_ms();

    if ((now_ms - grayscale_settle_start_ms) <
        GRAYSCALE_HW_SCAN_PERIOD_MS)
    {
        return;
    }

    grayscale_settle_start_ms = now_ms;
    if (0u == grayscale_hw_read_states(grayscale_work_values))
    {
        return;
    }

    /* Publish only a complete scan so OLED/control never tear. */
    for (i = 0u; i < GRAYSCALE_CHANNELS; i++)
    {
        grayscale_values[i] = grayscale_work_values[i];
    }
    grayscale_scan_ready = 1u;
    grayscale_scan_sequence++;
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
