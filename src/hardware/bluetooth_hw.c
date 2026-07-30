#include "bluetooth_hw.h"

#include "ti_msp_dl_config.h"

static volatile uint8 bluetooth_hw_rx_buffer[BLUETOOTH_HW_RX_BUFFER_SIZE];
static volatile uint16 bluetooth_hw_rx_head;
static volatile uint16 bluetooth_hw_rx_tail;

static uint8 bluetooth_hw_tx_buffer[BLUETOOTH_HW_TX_BUFFER_SIZE];
static uint16 bluetooth_hw_tx_head;
static uint16 bluetooth_hw_tx_tail;

static volatile uint32 bluetooth_hw_rx_count;
static volatile uint32 bluetooth_hw_rx_overflow_count;
static uint32 bluetooth_hw_tx_drop_count;

static uint16 bluetooth_hw_next_index(uint16 index, uint16 size)
{
    return (uint16)((index + 1u) % size);
}

void bluetooth_hw_init(void)
{
    bluetooth_hw_rx_head = 0u;
    bluetooth_hw_rx_tail = 0u;
    bluetooth_hw_tx_head = 0u;
    bluetooth_hw_tx_tail = 0u;
    bluetooth_hw_rx_count = 0u;
    bluetooth_hw_rx_overflow_count = 0u;
    bluetooth_hw_tx_drop_count = 0u;

    DL_UART_Main_clearInterruptStatus(UART_BLUETOOTH_INST,
                                      DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(UART_BLUETOOTH_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    NVIC_SetPriority(UART_BLUETOOTH_INST_INT_IRQN, 2u);
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
}

uint8 bluetooth_hw_read_byte(uint8 *data)
{
    uint16 tail;

    if ((NULL == data) || (bluetooth_hw_rx_head == bluetooth_hw_rx_tail))
    {
        return 0u;
    }

    tail = bluetooth_hw_rx_tail;
    *data = bluetooth_hw_rx_buffer[tail];
    bluetooth_hw_rx_tail = bluetooth_hw_next_index(tail,
                                                    BLUETOOTH_HW_RX_BUFFER_SIZE);
    return 1u;
}

uint16 bluetooth_hw_write(const uint8 *data, uint16 length)
{
    uint16 written = 0u;

    if (NULL == data)
    {
        return 0u;
    }

    while (written < length)
    {
        uint16 next = bluetooth_hw_next_index(bluetooth_hw_tx_head,
                                              BLUETOOTH_HW_TX_BUFFER_SIZE);
        if (next == bluetooth_hw_tx_tail)
        {
            bluetooth_hw_tx_drop_count += (uint32)(length - written);
            break;
        }

        bluetooth_hw_tx_buffer[bluetooth_hw_tx_head] = data[written];
        bluetooth_hw_tx_head = next;
        written++;
    }

    return written;
}

void bluetooth_hw_send_string(const char *str)
{
    while ((NULL != str) && ('\0' != *str))
    {
        uint8 byte = (uint8)(*str);
        if (0u == bluetooth_hw_write(&byte, 1u))
        {
            break;
        }
        str++;
    }
}

void bluetooth_hw_tx_pump(void)
{
    while ((bluetooth_hw_tx_head != bluetooth_hw_tx_tail) &&
           (false == DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST)))
    {
        DL_UART_Main_transmitData(
            UART_BLUETOOTH_INST,
            bluetooth_hw_tx_buffer[bluetooth_hw_tx_tail]);
        bluetooth_hw_tx_tail = bluetooth_hw_next_index(
            bluetooth_hw_tx_tail, BLUETOOTH_HW_TX_BUFFER_SIZE);
    }
}

uint32 bluetooth_hw_get_rx_count(void)
{
    return bluetooth_hw_rx_count;
}

uint32 bluetooth_hw_get_rx_overflow_count(void)
{
    return bluetooth_hw_rx_overflow_count;
}

uint32 bluetooth_hw_get_tx_drop_count(void)
{
    return bluetooth_hw_tx_drop_count;
}

void UART3_IRQHandler(void)
{
    if (DL_UART_MAIN_IIDX_RX ==
        DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST))
    {
        uint8 data = DL_UART_Main_receiveData(UART_BLUETOOTH_INST);
        uint16 head = bluetooth_hw_rx_head;
        uint16 next = bluetooth_hw_next_index(head,
                                              BLUETOOTH_HW_RX_BUFFER_SIZE);

        bluetooth_hw_rx_count++;
        if (next != bluetooth_hw_rx_tail)
        {
            bluetooth_hw_rx_buffer[head] = data;
            bluetooth_hw_rx_head = next;
        }
        else
        {
            bluetooth_hw_rx_overflow_count++;
        }
    }
}
