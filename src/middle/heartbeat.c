#include <stdio.h>

#include "encoder.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "zf_common_typedef.h"

static volatile uint32 heartbeat_tick_count = 0;

static void heartbeat_send(void)
{
    char message[64];
    int32 left_rpm;
    int32 right_rpm;

    heartbeat_hw_led_toggle();

    heartbeat_tick_count ++;
    encoder_update_speed(HEARTBEAT_PERIOD_MS);
    left_rpm  = encoder_get_left_rpm();
    right_rpm = encoder_get_right_rpm();
    snprintf(message, sizeof(message), "[hb] %lu,%ld,%ld\r\n",
             (unsigned long)heartbeat_tick_count, (long)left_rpm, (long)right_rpm);
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

uint32 heartbeat_get_ms(void)
{
    return heartbeat_hw_get_ms();
}
