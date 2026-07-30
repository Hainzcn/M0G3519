#include "bluetooth_test_app.h"

#include "bluetooth_hw.h"
#include "heartbeat.h"
#include "heartbeat_hw.h"

#define BLUETOOTH_TEST_ALIVE_PERIOD_MS      (1000u)
#define BLUETOOTH_TEST_MAX_ECHO_BYTES       (64u)

static uint32 bluetooth_test_last_alive_ms;

void bluetooth_test_app_init(void)
{
    bluetooth_hw_init();
    bluetooth_test_last_alive_ms = heartbeat_get_ms();

    bluetooth_hw_send_string("[bt] ready,115200\r\n");
    heartbeat_hw_uart_send_string("[mode] bluetooth test\r\n");
}

void bluetooth_test_app_process(void)
{
    uint8 byte;
    uint8 echo_count = 0u;
    uint32 now_ms;

    bluetooth_hw_tx_pump();

    while ((echo_count < BLUETOOTH_TEST_MAX_ECHO_BYTES) &&
           (0u != bluetooth_hw_read_byte(&byte)))
    {
        (void)bluetooth_hw_write(&byte, 1u);
        echo_count++;
    }

    now_ms = heartbeat_get_ms();
    if ((now_ms - bluetooth_test_last_alive_ms) >=
        BLUETOOTH_TEST_ALIVE_PERIOD_MS)
    {
        bluetooth_test_last_alive_ms = now_ms;
        bluetooth_hw_send_string("[bt] alive\r\n");
    }

    bluetooth_hw_tx_pump();
}
