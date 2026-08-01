#include "power_app.h"

#include <stdio.h>

#include "heartbeat.h"
#include "heartbeat_hw.h"
#include "power.h"
#include "power_hw.h"

#define POWER_APP_DEBUG_PERIOD_MS  (200u)

static uint32 power_app_last_print_ms;

void power_app_init(void)
{
    power_init();
    power_app_last_print_ms = 0u;
}

void power_app_process(void)
{
    char message[80];
    uint32 now_ms;

    power_process();
    now_ms = heartbeat_get_ms();
    if ((now_ms - power_app_last_print_ms) < POWER_APP_DEBUG_PERIOD_MS)
    {
        return;
    }

    power_app_last_print_ms = now_ms;
    snprintf(message, sizeof(message), "[pwr] %u,ok=%u,mv=%u,raw=%u,e=%u\r\n",
             (unsigned int)now_ms,
             (unsigned int)power_is_valid(),
             (unsigned int)power_get_bus_millivolts(),
             (unsigned int)power_get_raw(),
             (unsigned int)power_hw_get_error_count());
    heartbeat_hw_uart_send_string(message);
}
