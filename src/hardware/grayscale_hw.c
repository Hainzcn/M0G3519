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
    ch &= 0x07u;

    gpio_set_level(GRAYSCALE_HW_AD0_PIN, (ch >> 0) & 0x01u);
    gpio_set_level(GRAYSCALE_HW_AD1_PIN, (ch >> 1) & 0x01u);
    gpio_set_level(GRAYSCALE_HW_AD2_PIN, (ch >> 2) & 0x01u);
}

uint8 grayscale_hw_read_out(void)
{
    return gpio_get_level(GRAYSCALE_HW_OUT_PIN);
}
