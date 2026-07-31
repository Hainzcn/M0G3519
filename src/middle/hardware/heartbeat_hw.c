#include "heartbeat_hw.h"

#include "motor.h"
#include "ti_msp_dl_config.h"

#include "zf_common_interrupt.h"
#include "zf_driver_gpio.h"

#define HEARTBEAT_HW_LED_PIN           (A14)
#define HEARTBEAT_HW_SYSTICK_PERIOD_MS (1)

static volatile uint32 heartbeat_hw_tick_ms = 0;
static volatile uint32 heartbeat_hw_pending_ticks = 0;
static volatile uint32 heartbeat_hw_ms = 0;
static volatile uint32 heartbeat_hw_sequence = 0;
static uint32 heartbeat_hw_period_ms = 0;

static uint8 heartbeat_hw_tx_buffer[HEARTBEAT_HW_TX_FIFO_SIZE];
static volatile uint16 heartbeat_hw_tx_head;
static volatile uint16 heartbeat_hw_tx_tail;
static volatile uint32 heartbeat_hw_tx_drop_count;

static uint16 heartbeat_hw_tx_next_index(uint16 index)
{
    return (uint16)((index + 1u) % HEARTBEAT_HW_TX_FIFO_SIZE);
}

static uint8 heartbeat_hw_tx_has_data(void)
{
    return (heartbeat_hw_tx_head != heartbeat_hw_tx_tail) ? 1u : 0u;
}

static void heartbeat_hw_uart_tx_send_one(void)
{
    uint16 tail;

    if (0u == heartbeat_hw_tx_has_data())
    {
        return;
    }

    if (true == DL_UART_Main_isTXFIFOFull(UART_0_INST))
    {
        return;
    }

    tail = heartbeat_hw_tx_tail;
    DL_UART_Main_transmitData(UART_0_INST, heartbeat_hw_tx_buffer[tail]);
    heartbeat_hw_tx_tail = heartbeat_hw_tx_next_index(tail);
}

static uint16 heartbeat_hw_uart_tx_free(void)
{
    uint16 head = heartbeat_hw_tx_head;
    uint16 tail = heartbeat_hw_tx_tail;

    if (head >= tail)
    {
        return (uint16)(HEARTBEAT_HW_TX_FIFO_SIZE -
                        (head - tail) - 1u);
    }
    return (uint16)(tail - head - 1u);
}

void heartbeat_hw_init(uint32 tick_period_ms)
{
    gpio_init(HEARTBEAT_HW_LED_PIN, GPO, 0, GPO_PUSH_PULL);

    heartbeat_hw_period_ms     = tick_period_ms;
    heartbeat_hw_tick_ms       = 0;
    heartbeat_hw_pending_ticks = 0;
    heartbeat_hw_ms            = 0;
    heartbeat_hw_sequence      = 0;
    heartbeat_hw_tx_head       = 0;
    heartbeat_hw_tx_tail       = 0;
    heartbeat_hw_tx_drop_count = 0;
    SysTick_Config(CPUCLK_FREQ / (1000 / HEARTBEAT_HW_SYSTICK_PERIOD_MS));
}

void heartbeat_hw_led_toggle(void)
{
    gpio_toggle_level(HEARTBEAT_HW_LED_PIN);
}

void heartbeat_hw_uart_send_string(const char *str)
{
    uint32 primask;
    uint16 length = 0u;
    uint16 head;
    uint16 index;

    if (NULL == str)
    {
        return;
    }
    while ((length < 0xFFFFu) && ('\0' != str[length]))
    {
        length++;
    }
    if (0u == length)
    {
        return;
    }

    primask = interrupt_global_disable();
    if (length > heartbeat_hw_uart_tx_free())
    {
        heartbeat_hw_tx_drop_count += length;
        interrupt_global_enable(primask);
        return;
    }

    head = heartbeat_hw_tx_head;
    for (index = 0u; index < length; index++)
    {
        heartbeat_hw_tx_buffer[head] = (uint8)str[index];
        head = heartbeat_hw_tx_next_index(head);
    }
    heartbeat_hw_tx_head = head;
    interrupt_global_enable(primask);
}

void heartbeat_hw_uart_tx_pump(void)
{
    while ((0u != heartbeat_hw_tx_has_data()) &&
           (false == DL_UART_Main_isTXFIFOFull(UART_0_INST)))
    {
        heartbeat_hw_uart_tx_send_one();
    }
}

void heartbeat_hw_uart_flush_blocking(void)
{
    while (0u != heartbeat_hw_tx_has_data())
    {
        heartbeat_hw_uart_tx_pump();
    }

    while (true == DL_UART_isBusy(UART_0_INST))
    {
    }
}

uint8 heartbeat_hw_uart_tx_busy(void)
{
    return ((0u != heartbeat_hw_tx_has_data()) ||
            (true == DL_UART_isBusy(UART_0_INST))) ? 1u : 0u;
}

uint32 heartbeat_hw_uart_get_drop_count(void)
{
    return heartbeat_hw_tx_drop_count;
}

uint8 heartbeat_hw_take_tick(void)
{
    uint8 has_tick = 0;
    uint32 primask;

    primask = interrupt_global_disable();
    if (0 != heartbeat_hw_pending_ticks)
    {
        heartbeat_hw_pending_ticks --;
        has_tick = 1;
    }
    interrupt_global_enable(primask);

    return has_tick;
}

uint32 heartbeat_hw_get_ms(void)
{
    return heartbeat_hw_ms;
}

uint32 heartbeat_hw_get_sequence(void)
{
    return heartbeat_hw_sequence;
}

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
        heartbeat_hw_sequence ++;
    }

    motor_watchdog_check();
}
