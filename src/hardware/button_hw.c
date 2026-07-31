#include "button_hw.h"

#include "zf_driver_gpio.h"

typedef struct
{
    gpio_pin_enum pin;
} button_hw_entry_t;

static const button_hw_entry_t button_hw_table[BUTTON_HW_COUNT] =
{
    { A30 },    /* SW1 */
    { A31 },    /* SW2 */
    { B1  },    /* SW3 */
    { B0  },    /* SW4 */
};

void button_hw_init(void)
{
    uint8 i;

    for (i = 0u; i < BUTTON_HW_COUNT; i++)
    {
        gpio_init(button_hw_table[i].pin, GPI, GPIO_HIGH, GPI_PULL_UP);
    }
}

uint8 button_hw_read_raw(uint8 index)
{
    if (index >= BUTTON_HW_COUNT)
    {
        return (uint8)(BUTTON_HW_ACTIVE_LEVEL ^ 1u);
    }

    return gpio_get_level(button_hw_table[index].pin);
}
