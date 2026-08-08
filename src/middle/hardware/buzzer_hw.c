#include "buzzer_hw.h"

#include "zf_driver_gpio.h"

#define BUZZER_HW_GPIO_PIN    (A13)

void buzzer_hw_init(void)
{
    gpio_init(BUZZER_HW_GPIO_PIN, GPO,
              (uint8)(BUZZER_HW_ACTIVE_LEVEL ^ 1u), GPO_PUSH_PULL);
}

void buzzer_hw_set_enabled(uint8 enabled)
{
    uint8 level = (0u != enabled) ? BUZZER_HW_ACTIVE_LEVEL :
                                    (uint8)(BUZZER_HW_ACTIVE_LEVEL ^ 1u);

    gpio_set_level(BUZZER_HW_GPIO_PIN, level);
}
