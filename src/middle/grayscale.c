#include "grayscale.h"

#include "grayscale_hw.h"
#include "heartbeat.h"

#define GRAYSCALE_POLL_PERIOD_MS    (2u)
#define GRAYSCALE_ONLINE_TIMEOUT_MS (20u)

static uint8 grayscale_values[GRAYSCALE_CHANNELS];
static uint8 grayscale_scan_ready;
static uint8 grayscale_read_pending;
static uint32 grayscale_scan_sequence;
static uint32 grayscale_last_request_ms;
static uint32 grayscale_last_update_ms;
static uint8 grayscale_raw_bits;

void grayscale_init(void)
{
    uint8 i;

    grayscale_hw_init();
    for (i = 0u; i < GRAYSCALE_CHANNELS; i++)
    {
        grayscale_values[i] = 0u;
    }
    grayscale_scan_ready = 0u;
    grayscale_read_pending = 0u;
    grayscale_scan_sequence = 0u;
    grayscale_last_request_ms = heartbeat_get_ms() - GRAYSCALE_POLL_PERIOD_MS;
    grayscale_last_update_ms = 0u;
    grayscale_raw_bits = 0u;
}

void grayscale_process(void)
{
    uint8 channel;
    uint8 read_result;
    uint8 sensor_bits;
    uint32 now_ms;

    grayscale_hw_process();
    if (0u != grayscale_read_pending)
    {
        read_result = grayscale_hw_take_read(&sensor_bits);
        if (0u == read_result)
        {
            return;
        }

        grayscale_read_pending = 0u;
        if (1u != read_result)
        {
            return;
        }

        /* X1 is bit 7 through X8 bit 0; module output is active-low. */
        for (channel = 0u; channel < GRAYSCALE_CHANNELS; channel++)
        {
            grayscale_values[channel] =
                (uint8)(((sensor_bits >> (7u - channel)) & 0x01u) ^ 0x01u);
        }
        grayscale_raw_bits = sensor_bits;
        grayscale_last_update_ms = heartbeat_get_ms();
        grayscale_scan_ready = 1u;
        grayscale_scan_sequence++;
    }

    now_ms = heartbeat_get_ms();
    if ((now_ms - grayscale_last_request_ms) < GRAYSCALE_POLL_PERIOD_MS)
    {
        return;
    }
    if (0u != grayscale_hw_start_read())
    {
        grayscale_read_pending = 1u;
        grayscale_last_request_ms = now_ms;
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

uint8 grayscale_get_raw_bits(void)
{
    return grayscale_raw_bits;
}

uint8 grayscale_is_online(void)
{
    if (0u == grayscale_scan_sequence)
    {
        return 0u;
    }
    return ((heartbeat_get_ms() - grayscale_last_update_ms) <=
            GRAYSCALE_ONLINE_TIMEOUT_MS) ? 1u : 0u;
}

uint32 grayscale_get_last_update_ms(void)
{
    return grayscale_last_update_ms;
}

uint32 grayscale_get_error_count(void)
{
    return grayscale_hw_get_error_count();
}
