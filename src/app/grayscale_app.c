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
    char message[64];
    const uint8 *values;
    uint32 now_ms;
    uint32 error_count;
    uint8 has_new_frame;

    grayscale_process();
    has_new_frame = grayscale_take_scan_ready();

    now_ms = heartbeat_get_ms();
    if ((now_ms - grayscale_app_last_print_ms) < GRAYSCALE_APP_DEBUG_PERIOD_MS)
    {
        return;
    }

    grayscale_app_last_print_ms = now_ms;
    grayscale_app_print_count ++;
    values = grayscale_get_values();
    error_count = grayscale_hw_get_error_count();
    snprintf(message, sizeof(message),
             "[gs] %u,ok=%u,v=%u%u%u%u%u%u,e=%u\r\n",
             (unsigned int)grayscale_app_print_count,
             (unsigned int)has_new_frame,
             (unsigned int)values[0], (unsigned int)values[1],
             (unsigned int)values[2], (unsigned int)values[3],
             (unsigned int)values[4], (unsigned int)values[5],
             (unsigned int)error_count);
    heartbeat_hw_uart_send_string(message);
}
