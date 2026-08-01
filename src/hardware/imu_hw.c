#include "imu_hw.h"

#include "ti_msp_dl_config.h"

static uint8 imu_hw_rx_fifo[IMU_HW_RX_FIFO_SIZE];
static volatile uint16 imu_hw_rx_write_index;
static volatile uint16 imu_hw_rx_read_index;
static volatile uint32 imu_hw_rx_overflow_count;
static volatile uint8 imu_hw_rx_error;
static uint8 imu_hw_rx_irq_enabled;

static uint16 imu_hw_next_index(uint16 index)
{
    index++;
    return (index >= IMU_HW_RX_FIFO_SIZE) ? 0u : index;
}

static void imu_hw_push_byte(uint8 byte)
{
    uint16 next = imu_hw_next_index(imu_hw_rx_write_index);

    if (next == imu_hw_rx_read_index)
    {
        imu_hw_rx_overflow_count++;
        imu_hw_rx_error = 1u;
        return;
    }

    imu_hw_rx_fifo[imu_hw_rx_write_index] = byte;
    imu_hw_rx_write_index = next;
}

static uint8 imu_hw_wait_tx_ready(void)
{
    uint32 timeout = IMU_HW_TX_TIMEOUT_CYCLES;

    while ((true == DL_UART_isBusy(UART_1_INST)) && (0u != timeout))
    {
        timeout--;
    }

    return (0u != timeout) ? 1u : 0u;
}

void imu_hw_init(void)
{
    imu_hw_rx_write_index = 0u;
    imu_hw_rx_read_index = 0u;
    imu_hw_rx_overflow_count = 0u;
    imu_hw_rx_error = 0u;
    imu_hw_rx_irq_enabled = 0u;
}

void imu_hw_rx_enable(void)
{
    if (0u != imu_hw_rx_irq_enabled)
    {
        return;
    }

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_SetPriority(UART_1_INST_INT_IRQN, 2);
    DL_UART_Main_enableInterrupt(UART_1_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    imu_hw_rx_irq_enabled = 1u;
}

uint8 imu_hw_read_byte(uint8 *byte)
{
    if ((NULL == byte) ||
        (imu_hw_rx_read_index == imu_hw_rx_write_index))
    {
        return 0u;
    }

    *byte = imu_hw_rx_fifo[imu_hw_rx_read_index];
    imu_hw_rx_read_index = imu_hw_next_index(imu_hw_rx_read_index);
    return 1u;
}

uint8 imu_hw_write_frame(const uint8 *frame, uint8 len)
{
    uint8 i;

    if (NULL == frame)
    {
        return 0u;
    }

    for (i = 0u; i < len; i++)
    {
        if (!imu_hw_wait_tx_ready())
        {
            return 0u;
        }
        DL_UART_Main_transmitData(UART_1_INST, frame[i]);
    }

    return 1u;
}

void UART1_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST))
    {
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
        case DL_UART_MAIN_IIDX_RX:
            while (false == DL_UART_Main_isRXFIFOEmpty(UART_1_INST))
            {
                imu_hw_push_byte(DL_UART_Main_receiveData(UART_1_INST));
            }
            break;

        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_BREAK_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            (void)DL_UART_Main_getErrorStatus(UART_1_INST,
                DL_UART_MAIN_ERROR_OVERRUN | DL_UART_MAIN_ERROR_BREAK |
                DL_UART_MAIN_ERROR_PARITY | DL_UART_MAIN_ERROR_FRAMING);
            imu_hw_rx_error = 1u;
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
