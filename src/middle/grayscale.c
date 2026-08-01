#include "grayscale.h"

#include "grayscale_hw.h"
#include "heartbeat.h"

#define GRAYSCALE_STABLE_SAMPLES  (2u)

static uint8 grayscale_values[GRAYSCALE_CHANNELS];
static uint8 grayscale_work_values[GRAYSCALE_CHANNELS];
static uint8 grayscale_candidate_values[GRAYSCALE_CHANNELS];
static uint8 grayscale_scan_ready;
static uint8 grayscale_candidate_count;
static uint32 grayscale_scan_sequence;
static uint32 grayscale_settle_start_ms;

static uint8 grayscale_values_equal(const uint8 first[GRAYSCALE_CHANNELS],
                                    const uint8 second[GRAYSCALE_CHANNELS])
{
    uint8 i;

    for (i = 0u; i < GRAYSCALE_CHANNELS; i++)
    {
        if (first[i] != second[i])
        {
            return 0u;
        }
    }
    return 1u;
}

static void grayscale_copy_values(uint8 destination[GRAYSCALE_CHANNELS],
                                  const uint8 source[GRAYSCALE_CHANNELS])
{
    uint8 i;

    for (i = 0u; i < GRAYSCALE_CHANNELS; i++)
    {
        destination[i] = source[i];
    }
}

void grayscale_init(void)
{
    uint8 i;

    grayscale_hw_init();

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        grayscale_values[i] = 0;
        grayscale_work_values[i] = 0;
        grayscale_candidate_values[i] = 0;
    }

    grayscale_scan_ready = 0;
    grayscale_candidate_count = 0u;
    grayscale_scan_sequence = 0u;
    grayscale_settle_start_ms = heartbeat_get_ms() -
        GRAYSCALE_HW_SCAN_PERIOD_MS;
}

void grayscale_process(void)
{
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

    if (0u == grayscale_values_equal(grayscale_work_values,
                                     grayscale_candidate_values))
    {
        if (grayscale_candidate_count < GRAYSCALE_STABLE_SAMPLES)
        {
            grayscale_candidate_count++;
        }
    }
    else
    {
        grayscale_copy_values(grayscale_candidate_values,
                              grayscale_work_values);
        grayscale_candidate_count = 1u;
    }

    if (grayscale_candidate_count < GRAYSCALE_STABLE_SAMPLES)
    {
        return;
    }

    /* Publish only a complete, debounced scan so control never tears. */
    grayscale_copy_values(grayscale_values, grayscale_candidate_values);
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
