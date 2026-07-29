#include "grayscale_hw.h"

#include "zf_driver_gpio.h"

void grayscale_hw_init(void)
{
    gpio_init(GRAYSCALE_HW_AD0_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAYSCALE_HW_AD1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAYSCALE_HW_AD2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAYSCALE_HW_OUT_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
}

void grayscale_hw_select_channel(uint8 ch)
{
    uint32 address_value;
    const uint32 address_mask =
        DL_GPIO_PIN_15 | DL_GPIO_PIN_16 | DL_GPIO_PIN_17;

    ch &= 0x07u;
    address_value = ((0u != (ch & 0x01u)) ? DL_GPIO_PIN_15 : 0u) |
                    ((0u != (ch & 0x02u)) ? DL_GPIO_PIN_16 : 0u) |
                    ((0u != (ch & 0x04u)) ? DL_GPIO_PIN_17 : 0u);

    DL_GPIO_writePinsVal(GPIOA, address_mask, address_value);
}

uint8 grayscale_hw_read_out(void)
{
    return gpio_get_level(GRAYSCALE_HW_OUT_PIN);
}
