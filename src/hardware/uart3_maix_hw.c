#include "uart3_maix_hw.h"

#include "ti_msp_dl_config.h"

static volatile uint8 uart3_maix_hw_rx_buffer[UART3_MAIX_HW_RX_BUFFER_SIZE];
static volatile uint16 uart3_maix_hw_rx_head;
static volatile uint16 uart3_maix_hw_rx_tail;

static uint8 uart3_maix_hw_tx_buffer[UART3_MAIX_HW_TX_BUFFER_SIZE];
static uint16 uart3_maix_hw_tx_head;
static uint16 uart3_maix_hw_tx_tail;

static volatile uint32 uart3_maix_hw_rx_count;
static volatile uint32 uart3_maix_hw_rx_overflow_count;
static uint32 uart3_maix_hw_tx_drop_count;

static uint16 uart3_maix_hw_next_index(uint16 index, uint16 size)
{
    return (uint16)((index + 1u) % size);
}

void uart3_maix_hw_init(void)
{
    uart3_maix_hw_rx_head = 0u;
    uart3_maix_hw_rx_tail = 0u;
    uart3_maix_hw_tx_head = 0u;
    uart3_maix_hw_tx_tail = 0u;
    uart3_maix_hw_rx_count = 0u;
    uart3_maix_hw_rx_overflow_count = 0u;
    uart3_maix_hw_tx_drop_count = 0u;

    DL_UART_Main_clearInterruptStatus(UART_BLUETOOTH_INST,
                                      DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(UART_BLUETOOTH_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    NVIC_SetPriority(UART_BLUETOOTH_INST_INT_IRQN, 2u);
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
}

uint8 uart3_maix_hw_read_byte(uint8 *data)
{
    uint16 tail;

    if ((NULL == data) || (uart3_maix_hw_rx_head == uart3_maix_hw_rx_tail))
    {
        return 0u;
    }

    tail = uart3_maix_hw_rx_tail;
    *data = uart3_maix_hw_rx_buffer[tail];
    uart3_maix_hw_rx_tail = uart3_maix_hw_next_index(tail,
                                                    UART3_MAIX_HW_RX_BUFFER_SIZE);
    return 1u;
}

uint16 uart3_maix_hw_write(const uint8 *data, uint16 length)
{
    uint16 written = 0u;

    if (NULL == data)
    {
        return 0u;
    }

    while (written < length)
    {
        uint16 next = uart3_maix_hw_next_index(uart3_maix_hw_tx_head,
                                              UART3_MAIX_HW_TX_BUFFER_SIZE);
        if (next == uart3_maix_hw_tx_tail)
        {
            uart3_maix_hw_tx_drop_count += (uint32)(length - written);
            break;
        }

        uart3_maix_hw_tx_buffer[uart3_maix_hw_tx_head] = data[written];
        uart3_maix_hw_tx_head = next;
        written++;
    }

    return written;
}

uint16 uart3_maix_hw_write_atomic(const uint8 *data, uint16 length)
{
    uint16 used;
    uint16 free_bytes;

    if ((NULL == data) || (0u == length) ||
        (length >= UART3_MAIX_HW_TX_BUFFER_SIZE))
    {
        return 0u;
    }

    if (uart3_maix_hw_tx_head >= uart3_maix_hw_tx_tail)
    {
        used = (uint16)(uart3_maix_hw_tx_head - uart3_maix_hw_tx_tail);
    }
    else
    {
        used = (uint16)(UART3_MAIX_HW_TX_BUFFER_SIZE -
                        uart3_maix_hw_tx_tail + uart3_maix_hw_tx_head);
    }
    free_bytes = (uint16)(UART3_MAIX_HW_TX_BUFFER_SIZE - 1u - used);
    if (free_bytes < length)
    {
        uart3_maix_hw_tx_drop_count += length;
        return 0u;
    }

    return uart3_maix_hw_write(data, length);
}

void uart3_maix_hw_send_string(const char *str)
{
    while ((NULL != str) && ('\0' != *str))
    {
        uint8 byte = (uint8)(*str);
        if (0u == uart3_maix_hw_write(&byte, 1u))
        {
            break;
        }
        str++;
    }
}

void uart3_maix_hw_tx_pump(void)
{
    while ((uart3_maix_hw_tx_head != uart3_maix_hw_tx_tail) &&
           (false == DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST)))
    {
        DL_UART_Main_transmitData(
            UART_BLUETOOTH_INST,
            uart3_maix_hw_tx_buffer[uart3_maix_hw_tx_tail]);
        uart3_maix_hw_tx_tail = uart3_maix_hw_next_index(
            uart3_maix_hw_tx_tail, UART3_MAIX_HW_TX_BUFFER_SIZE);
    }
}

uint32 uart3_maix_hw_get_rx_count(void)
{
    return uart3_maix_hw_rx_count;
}

uint32 uart3_maix_hw_get_rx_overflow_count(void)
{
    return uart3_maix_hw_rx_overflow_count;
}

uint32 uart3_maix_hw_get_tx_drop_count(void)
{
    return uart3_maix_hw_tx_drop_count;
}

void UART3_IRQHandler(void)
{
    if (DL_UART_MAIN_IIDX_RX ==
        DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST))
    {
        uint8 data = DL_UART_Main_receiveData(UART_BLUETOOTH_INST);
        uint16 head = uart3_maix_hw_rx_head;
        uint16 next = uart3_maix_hw_next_index(head,
                                              UART3_MAIX_HW_RX_BUFFER_SIZE);

        uart3_maix_hw_rx_count++;
        if (next != uart3_maix_hw_rx_tail)
        {
            uart3_maix_hw_rx_buffer[head] = data;
            uart3_maix_hw_rx_head = next;
        }
        else
        {
            uart3_maix_hw_rx_overflow_count++;
        }
    }
}
