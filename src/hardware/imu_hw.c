#include "imu_hw.h"

#include "ti_msp_dl_config.h"

static uint8 imu_hw_rx_buffer[IMU_HW_RX_FIFO_SIZE];
static volatile uint16 imu_hw_rx_head;
static volatile uint16 imu_hw_rx_tail;
static volatile uint32 imu_hw_rx_overflow_count;
static volatile uint8 imu_hw_rx_error;
static uint8 imu_hw_rx_irq_enabled;

static uint8 imu_hw_wait_tx_ready(void)
{
    uint32 timeout = IMU_HW_TX_TIMEOUT_CYCLES;

    while ((true == DL_UART_isBusy(UART_1_INST)) && (0u != timeout))
    {
        timeout --;
    }

    return (0u != timeout) ? 1u : 0u;
}

void imu_hw_init(void)
{
    imu_hw_rx_head            = 0;
    imu_hw_rx_tail            = 0;
    imu_hw_rx_overflow_count  = 0;
    imu_hw_rx_error           = 0;
    imu_hw_rx_irq_enabled     = 0;
}

void imu_hw_rx_enable(void)
{
    if (0u != imu_hw_rx_irq_enabled)
    {
        return;
    }

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_SetPriority(UART_1_INST_INT_IRQN, 2);
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    imu_hw_rx_irq_enabled = 1;
}

uint8 imu_hw_read_byte(uint8 *byte)
{
    uint16 head;
    uint16 tail;

    if (NULL == byte)
    {
        return 0;
    }

    head = imu_hw_rx_head;
    tail = imu_hw_rx_tail;
    if (tail == head)
    {
        return 0;
    }

    *byte = imu_hw_rx_buffer[tail];
    imu_hw_rx_tail = (uint16)((tail + 1u) % IMU_HW_RX_FIFO_SIZE);
    return 1;
}

uint8 imu_hw_write_frame(const uint8 *frame, uint8 len)
{
    uint8 i;

    if (NULL == frame)
    {
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        if (!imu_hw_wait_tx_ready())
        {
            return 0;
        }
        DL_UART_Main_transmitData(UART_1_INST, frame[i]);
    }

    return 1;
}

uint8 imu_hw_write_reg(uint8 addr, int16 value)
{
    uint8 frame[5];

    frame[0] = 0x55;
    frame[1] = 0xAA;
    frame[2] = addr;
    frame[3] = (uint8)(value & 0xFF);
    frame[4] = (uint8)((value >> 8) & 0xFF);

    return imu_hw_write_frame(frame, 5);
}

void UART1_IRQHandler(void)
{
    uint8 rx_byte;
    uint16 head;
    uint16 next;

    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (false == DL_UART_isRXFIFOEmpty(UART_1_INST))
            {
                rx_byte = (uint8)DL_UART_Main_receiveData(UART_1_INST);
                head    = imu_hw_rx_head;
                next    = (uint16)((head + 1u) % IMU_HW_RX_FIFO_SIZE);

                if (next != imu_hw_rx_tail)
                {
                    imu_hw_rx_buffer[head] = rx_byte;
                    imu_hw_rx_head           = next;
                }
                else
                {
                    imu_hw_rx_overflow_count ++;
                    imu_hw_rx_error = 1u;
                }
            }
            break;

        default:
            break;
    }
}

uint32 imu_hw_get_overflow_count(void)
{
    return imu_hw_rx_overflow_count;
}

uint8 imu_hw_take_rx_error(void)
{
    if (0u == imu_hw_rx_error)
    {
        return 0u;
    }

    imu_hw_rx_error = 0u;
    return 1u;
}
