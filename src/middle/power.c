#include "power.h"

#include "heartbeat.h"
#include "power_hw.h"

#define POWER_SAMPLE_PERIOD_MS       (20u)
#define POWER_ADC_FULL_SCALE          (4095u)
#define POWER_ADC_REFERENCE_MV        (3300u)
#define POWER_DIVIDER_RATIO           (11u)
#define POWER_FILTER_DIVISOR          (4)

static uint16 power_raw;
static uint16 power_bus_millivolts;
static uint32 power_last_sample_ms;
static uint8 power_valid;

static uint16 power_convert_millivolts(uint16 raw)
{
    uint32 millivolts;

    millivolts = (uint32)raw * POWER_ADC_REFERENCE_MV *
                 POWER_DIVIDER_RATIO;
    millivolts = (millivolts + (POWER_ADC_FULL_SCALE / 2u)) /
                 POWER_ADC_FULL_SCALE;
    return (uint16)millivolts;
}

void power_init(void)
{
    power_hw_init();
    power_raw = 0u;
    power_bus_millivolts = 0u;
    power_last_sample_ms = heartbeat_get_ms() - POWER_SAMPLE_PERIOD_MS;
    power_valid = 0u;
}

void power_process(void)
{
    uint16 sample_millivolts;
    uint32 now_ms = heartbeat_get_ms();

    if ((now_ms - power_last_sample_ms) < POWER_SAMPLE_PERIOD_MS)
    {
        return;
    }

    power_last_sample_ms = now_ms;
    if (0u == power_hw_read_raw(&power_raw))
    {
        return;
    }
    sample_millivolts = power_convert_millivolts(power_raw);

    if (0u == power_valid)
    {
        power_bus_millivolts = sample_millivolts;
        power_valid = 1u;
    }
    else
    {
        power_bus_millivolts = (uint16)((int32)power_bus_millivolts +
            (((int32)sample_millivolts - power_bus_millivolts) /
             POWER_FILTER_DIVISOR));
    }
}

uint16 power_get_bus_millivolts(void)
{
    return power_bus_millivolts;
}

uint16 power_get_raw(void)
{
    return power_raw;
}

uint8 power_is_valid(void)
{
    return power_valid;
}
