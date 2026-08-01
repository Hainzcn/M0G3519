#include "power_hw.h"

#include "ti_msp_dl_config.h"
#include "zf_driver_gpio.h"

#define POWER_HW_ADC_CHANNEL              (1u)
#define POWER_HW_ADC_AVERAGE_SAMPLES       (16u)
#define POWER_HW_ADC_SAMPLE_CYCLES         (320u)
#define POWER_HW_ADC_TIMEOUT_CYCLES        (20000u)

static uint32 power_hw_error_count;

static uint8 power_hw_read_one(uint16 *raw)
{
    uint32 timeout = POWER_HW_ADC_TIMEOUT_CYCLES;

    DL_ADC12_clearInterruptStatus(ADC0,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_startConversion(ADC0);
    while (0u == DL_ADC12_getRawInterruptStatus(ADC0,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED))
    {
        if (0u == timeout)
        {
            DL_ADC12_stopConversion(ADC0);
            power_hw_error_count++;
            return 0u;
        }
        timeout--;
    }

    *raw = DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
    DL_ADC12_stopConversion(ADC0);
    return 1u;
}

void power_hw_init(void)
{
    static const DL_ADC12_ClockConfig clock_config =
    {
        .clockSel = DL_ADC12_CLOCK_HFCLK,
        .divideRatio = DL_ADC12_CLOCK_DIVIDE_2,
        .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_32_TO_40,
    };

    gpio_init(A26, GPI, GPIO_LOW, GPI_ANAOG_IN);
    DL_ADC12_enablePower(ADC0);
    DL_ADC12_disableConversions(ADC0);
    DL_ADC12_setClockConfig(ADC0, (DL_ADC12_ClockConfig *)&clock_config);
    DL_ADC12_setStartAddress(ADC0, DL_ADC12_SEQ_START_ADDR_00);
    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_0,
        POWER_HW_ADC_CHANNEL, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    /* The driver board's 100 kOhm / 10 kOhm divider needs settling time. */
    DL_ADC12_setSampleTime0(ADC0, POWER_HW_ADC_SAMPLE_CYCLES);
    ADC0->ULLMEM.CTL2 &= ~ADC12_CTL2_RES_MASK;
    ADC0->ULLMEM.CTL2 |= (ADC_12BIT << ADC12_CTL2_RES_OFS);
    ADC0->ULLMEM.CTL0 |= ADC12_CTL0_PWRDN_MANUAL;
    DL_ADC12_clearInterruptStatus(ADC0,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(ADC0);
    power_hw_error_count = 0u;
}

uint8 power_hw_read_raw(uint16 *raw)
{
    uint8 index;
    uint16 sample;
    uint32 sum = 0u;

    if (NULL == raw)
    {
        return 0u;
    }

    for (index = 0u; index < POWER_HW_ADC_AVERAGE_SAMPLES; index++)
    {
        if (0u == power_hw_read_one(&sample))
        {
            return 0u;
        }
        sum += sample;
    }

    *raw = (uint16)(sum / POWER_HW_ADC_AVERAGE_SAMPLES);
    return 1u;
}

uint32 power_hw_get_error_count(void)
{
    return power_hw_error_count;
}
