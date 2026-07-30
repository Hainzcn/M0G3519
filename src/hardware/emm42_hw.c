#include "emm42_hw.h"

#include "ti_msp_dl_config.h"

static volatile uint8 emm42_hw_rx_buffer[EMM42_HW_RX_BUFFER_SIZE];
static volatile uint16 emm42_hw_rx_head;
static volatile uint16 emm42_hw_rx_tail;
static volatile uint32 emm42_hw_rx_overflow_count;

static uint16 emm42_hw_next_index(uint16 index)
{
    return (uint16)((index + 1u) % EMM42_HW_RX_BUFFER_SIZE);
}

void emm42_hw_init(void)
{
    emm42_hw_rx_head = 0u;
    emm42_hw_rx_tail = 0u;
    emm42_hw_rx_overflow_count = 0u;

    DL_UART_Main_clearInterruptStatus(UART_EMM42_INST,
                                      DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(UART_EMM42_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_EMM42_INST_INT_IRQN);
    NVIC_SetPriority(UART_EMM42_INST_INT_IRQN, 2u);
    NVIC_EnableIRQ(UART_EMM42_INST_INT_IRQN);
}

uint8 emm42_hw_write(const uint8 *data, uint16 length)
{
    uint16 i;

    if ((NULL == data) || (0u == length))
    {
        return 0u;
    }

    for (i = 0u; i < length; i ++)
    {
        uint32 timeout = EMM42_HW_TX_TIMEOUT_LOOPS;

        while ((true == DL_UART_Main_isTXFIFOFull(UART_EMM42_INST)) &&
               (0u != timeout))
        {
            timeout --;
        }
        if (0u == timeout)
        {
            return 0u;
        }
        DL_UART_Main_transmitData(UART_EMM42_INST, data[i]);
    }

    return 1u;
}

uint8 emm42_hw_read_byte(uint8 *data)
{
    uint16 tail;

    if ((NULL == data) || (emm42_hw_rx_head == emm42_hw_rx_tail))
    {
        return 0u;
    }

    tail = emm42_hw_rx_tail;
    *data = emm42_hw_rx_buffer[tail];
    emm42_hw_rx_tail = emm42_hw_next_index(tail);
    return 1u;
}

void emm42_hw_clear_rx(void)
{
    emm42_hw_rx_tail = emm42_hw_rx_head;
}

uint32 emm42_hw_get_rx_overflow_count(void)
{
    return emm42_hw_rx_overflow_count;
}

void UART7_IRQHandler(void)
{
    if (DL_UART_MAIN_IIDX_RX ==
        DL_UART_Main_getPendingInterrupt(UART_EMM42_INST))
    {
        uint8 data = DL_UART_Main_receiveData(UART_EMM42_INST);
        uint16 head = emm42_hw_rx_head;
        uint16 next = emm42_hw_next_index(head);

        if (next != emm42_hw_rx_tail)
        {
            emm42_hw_rx_buffer[head] = data;
            emm42_hw_rx_head = next;
        }
        else
        {
            emm42_hw_rx_overflow_count ++;
        }
    }
}
