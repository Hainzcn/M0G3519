#include "grayscale_app.h"

#include <stdio.h>

#include "grayscale.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"

#define GRAYSCALE_APP_DEBUG_PERIOD_MS   (1000)

static uint32 grayscale_app_last_print_ms;
static uint32 grayscale_app_print_count;

void grayscale_app_init(void)
{
    grayscale_init();
    grayscale_app_last_print_ms = 0;
    grayscale_app_print_count   = 0;
}

void grayscale_app_process(void)
{
    char message[112];
    const uint8 *values;
    uint32 now_ms;
    uint32 age_ms;

    grayscale_process();
    now_ms = heartbeat_get_ms();
    if ((now_ms - grayscale_app_last_print_ms) < GRAYSCALE_APP_DEBUG_PERIOD_MS)
    {
        return;
    }

    grayscale_app_last_print_ms = now_ms;
    grayscale_app_print_count ++;
    values = grayscale_get_values();
    age_ms = (0u == grayscale_get_scan_sequence()) ?
        0xFFFFFFFFu : (now_ms - grayscale_get_last_update_ms());
    snprintf(message, sizeof(message),
             "[gs] %u,on=%u,raw=%02X,v=%u%u%u%u%u%u%u%u,err=%u,age=%d\r\n",
             (unsigned int)grayscale_app_print_count,
             (unsigned int)grayscale_is_online(),
             (unsigned int)grayscale_get_raw_bits(),
             (unsigned int)values[0], (unsigned int)values[1],
             (unsigned int)values[2], (unsigned int)values[3],
             (unsigned int)values[4], (unsigned int)values[5],
             (unsigned int)values[6], (unsigned int)values[7],
             (unsigned int)grayscale_get_error_count(),
             (int)age_ms);
    heartbeat_hw_uart_send_string(message);
}
