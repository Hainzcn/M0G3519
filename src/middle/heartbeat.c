#include <stdio.h>

#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "zf_common_typedef.h"

static volatile uint32 heartbeat_tick_count = 0;

static void heartbeat_send(void)
{
    char message[48];

    heartbeat_hw_led_toggle();

    heartbeat_tick_count ++;
    snprintf(message, sizeof(message), "HEARTBEAT,%lu\r\n", (unsigned long)heartbeat_tick_count);
    heartbeat_hw_uart_send_string(message);
}

void heartbeat_init(void)
{
    heartbeat_hw_init(HEARTBEAT_PERIOD_MS);
    heartbeat_hw_uart_send_string("BOOT OK\r\n");                      // 上电立即发一条，便于确认串口链路
}

void heartbeat_process(void)
{
    if (heartbeat_hw_take_tick())
    {
        heartbeat_send();
    }
}
