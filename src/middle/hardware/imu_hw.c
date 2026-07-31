#include "imu_hw.h"

#include "ti_msp_dl_config.h"

typedef enum
{
    IMU_HW_BLOCK_FREE = 0,
    IMU_HW_BLOCK_DMA,
    IMU_HW_BLOCK_READY,
    IMU_HW_BLOCK_READING,
} imu_hw_block_state_enum;

static uint8 imu_hw_rx_blocks[IMU_HW_DMA_BLOCK_COUNT][IMU_HW_DMA_BLOCK_SIZE];
static volatile uint8 imu_hw_block_state[IMU_HW_DMA_BLOCK_COUNT];
static volatile uint8 imu_hw_dma_block;
static volatile uint8 imu_hw_ready_read_index;
static volatile uint32 imu_hw_rx_overflow_count;
static volatile uint32 imu_hw_dma_block_count;
static volatile uint8 imu_hw_rx_error;
static uint8 imu_hw_rx_irq_enabled;

static void imu_hw_arm_dma(uint8 block)
{
    imu_hw_dma_block = block;
    imu_hw_block_state[block] = IMU_HW_BLOCK_DMA;
    DL_DMA_setSrcAddr(DMA, DMA_IMU_RX_CHAN_ID,
                      (uint32)(&UART_1_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_IMU_RX_CHAN_ID,
                       (uint32)&imu_hw_rx_blocks[block][0]);
    DL_DMA_setTransferSize(DMA, DMA_IMU_RX_CHAN_ID,
                           IMU_HW_DMA_BLOCK_SIZE);
    DL_DMA_enableChannel(DMA, DMA_IMU_RX_CHAN_ID);
}

static uint8 imu_hw_find_free_block(uint8 after)
{
    uint8 offset;

    for (offset = 1u; offset <= IMU_HW_DMA_BLOCK_COUNT; offset++)
    {
        uint8 block = (uint8)((after + offset) % IMU_HW_DMA_BLOCK_COUNT);
        if (IMU_HW_BLOCK_FREE == imu_hw_block_state[block])
        {
            return block;
        }
    }

    return IMU_HW_DMA_BLOCK_COUNT;
}

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
    uint8 block;

    for (block = 0u; block < IMU_HW_DMA_BLOCK_COUNT; block++)
    {
        imu_hw_block_state[block] = IMU_HW_BLOCK_FREE;
    }
    imu_hw_dma_block          = 0u;
    imu_hw_ready_read_index   = 0u;
    imu_hw_rx_overflow_count  = 0;
    imu_hw_dma_block_count    = 0;
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
    DL_UART_Main_enableInterrupt(UART_1_INST,
        DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    imu_hw_arm_dma(0u);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    imu_hw_rx_irq_enabled = 1;
}

const uint8 *imu_hw_acquire_block(uint16 *length)
{
    uint8 offset;

    if (NULL == length)
    {
        return NULL;
    }

    for (offset = 0u; offset < IMU_HW_DMA_BLOCK_COUNT; offset++)
    {
        uint8 block = (uint8)((imu_hw_ready_read_index + offset) %
                              IMU_HW_DMA_BLOCK_COUNT);
        if (IMU_HW_BLOCK_READY == imu_hw_block_state[block])
        {
            imu_hw_block_state[block] = IMU_HW_BLOCK_READING;
            imu_hw_ready_read_index =
                (uint8)((block + 1u) % IMU_HW_DMA_BLOCK_COUNT);
            *length = IMU_HW_DMA_BLOCK_SIZE;
            return &imu_hw_rx_blocks[block][0];
        }
    }

    *length = 0u;
    return NULL;
}

void imu_hw_release_block(const uint8 *block)
{
    uint8 index;

    for (index = 0u; index < IMU_HW_DMA_BLOCK_COUNT; index++)
    {
        if (block == &imu_hw_rx_blocks[index][0])
        {
            imu_hw_block_state[index] = IMU_HW_BLOCK_FREE;

            /* Resume after a queue overrun stopped the DMA channel. */
            if ((0u != imu_hw_rx_irq_enabled) &&
                (false == DL_DMA_isChannelEnabled(DMA,
                                                   DMA_IMU_RX_CHAN_ID)))
            {
                uint8 free_block = imu_hw_find_free_block(index);
                if (free_block < IMU_HW_DMA_BLOCK_COUNT)
                {
                    imu_hw_arm_dma(free_block);
                }
            }
            break;
        }
    }
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

void UART1_IRQHandler(void)
{
    uint8 completed;
    uint8 next;

    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST))
    {
        case DL_UART_MAIN_IIDX_DMA_DONE_RX:
            completed = imu_hw_dma_block;
            imu_hw_block_state[completed] = IMU_HW_BLOCK_READY;
            imu_hw_dma_block_count++;

            next = imu_hw_find_free_block(completed);
            if (next < IMU_HW_DMA_BLOCK_COUNT)
            {
                imu_hw_arm_dma(next);
            }
            else
            {
                imu_hw_rx_overflow_count++;
                imu_hw_rx_error = 1u;
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

uint32 imu_hw_get_dma_block_count(void)
{
    return imu_hw_dma_block_count;
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
