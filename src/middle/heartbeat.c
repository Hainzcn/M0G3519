#include <stdio.h>

#include "encoder.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "zf_common_typedef.h"

static void heartbeat_send(void)
{
    char message[64];
    int32 left_rpm;
    int32 right_rpm;
    uint32 sequence;

    heartbeat_hw_led_toggle();

    left_rpm  = encoder_get_left_rpm();
    right_rpm = encoder_get_right_rpm();
    sequence  = heartbeat_hw_get_sequence();
    snprintf(message, sizeof(message), "[hb] %u,%d,%d\r\n",
             (unsigned int)sequence, (int)left_rpm, (int)right_rpm);
    heartbeat_hw_uart_send_string(message);
}

void heartbeat_init(void)
{
    heartbeat_hw_init(HEARTBEAT_PERIOD_MS);
    heartbeat_hw_uart_send_string("BOOT OK\r\n");
    heartbeat_hw_uart_flush_blocking();
}

void heartbeat_process(void)
{
    if (heartbeat_hw_take_tick())
    {
        heartbeat_send();
    }

    heartbeat_hw_uart_tx_pump();
}

uint32 heartbeat_get_ms(void)
{
    return heartbeat_hw_get_ms();
}
