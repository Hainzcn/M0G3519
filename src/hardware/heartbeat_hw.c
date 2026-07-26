#include "heartbeat_hw.h"

#include "ti_msp_dl_config.h"

#include "zf_driver_gpio.h"

#define HEARTBEAT_HW_LED_PIN           (A14)
#define HEARTBEAT_HW_SYSTICK_PERIOD_MS (1)

static volatile uint32 heartbeat_hw_tick_ms = 0;
static volatile uint32 heartbeat_hw_pending_ticks = 0;
static volatile uint32 heartbeat_hw_ms = 0;
static uint32 heartbeat_hw_period_ms = 0;

void heartbeat_hw_init(uint32 tick_period_ms)
{
    gpio_init(HEARTBEAT_HW_LED_PIN, GPO, 0, GPO_PUSH_PULL);            // 状态灯默认熄灭

    // UART0 引脚/波特率由 SysConfig 在 clock_init()->SYSCFG_DL_init() 中完成，此处不再调用 zf uart_init
    heartbeat_hw_period_ms = tick_period_ms;
    heartbeat_hw_tick_ms = 0;
    heartbeat_hw_pending_ticks = 0;
    heartbeat_hw_ms = 0;
    SysTick_Config(CPUCLK_FREQ / (1000 / HEARTBEAT_HW_SYSTICK_PERIOD_MS));
}

void heartbeat_hw_led_toggle(void)
{
    gpio_toggle_level(HEARTBEAT_HW_LED_PIN);
}

void heartbeat_hw_uart_send_string(const char *str)
{
    while ((NULL != str) && ('\0' != *str))
    {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8) *str);
        str ++;
    }
}

uint8 heartbeat_hw_take_tick(void)
{
    uint8 has_tick = 0;

    __disable_irq();
    if (0 != heartbeat_hw_pending_ticks)
    {
        heartbeat_hw_pending_ticks --;
        has_tick = 1;
    }
    __enable_irq();

    return has_tick;
}

uint32 heartbeat_hw_get_ms(void)
{
    return heartbeat_hw_ms;
}

// SysTick 中断仅累积周期标志；不执行 UART 发送等可能阻塞的操作。
void SysTick_Handler(void)
{
    heartbeat_hw_ms ++;
    heartbeat_hw_tick_ms ++;
    if (heartbeat_hw_period_ms <= heartbeat_hw_tick_ms)
    {
        heartbeat_hw_tick_ms = 0;
        if (0xFFFFFFFFu != heartbeat_hw_pending_ticks)
        {
            heartbeat_hw_pending_ticks ++;
        }
    }
}
