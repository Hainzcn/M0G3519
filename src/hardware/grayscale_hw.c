#include "grayscale_hw.h"

#include "zf_driver_gpio.h"

void grayscale_hw_init(void)
{
    gpio_init(GRAYSCALE_HW_AD0_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAYSCALE_HW_AD1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAYSCALE_HW_AD2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAYSCALE_HW_OUT_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
    grayscale_hw_select_channel(0u);
}

void grayscale_hw_select_channel(uint8 ch)
{
    uint32 address_value;
    const uint32 address_mask =
        DL_GPIO_PIN_15 | DL_GPIO_PIN_16 | DL_GPIO_PIN_12;

    ch &= 0x07u;
    address_value = ((0u != (ch & 0x01u)) ? DL_GPIO_PIN_15 : 0u) |
                    ((0u != (ch & 0x02u)) ? DL_GPIO_PIN_16 : 0u) |
                    ((0u != (ch & 0x04u)) ? DL_GPIO_PIN_12 : 0u);

    /* Force all address bits low first, then set the required high bits. */
    DL_GPIO_clearPins(GPIOA, address_mask);
    if (0u != address_value)
    {
        DL_GPIO_setPins(GPIOA, address_value);
    }
}

uint8 grayscale_hw_read_out(void)
{
    return gpio_get_level(GRAYSCALE_HW_OUT_PIN);
}
