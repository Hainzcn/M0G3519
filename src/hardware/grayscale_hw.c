#include "grayscale_hw.h"

#include "oled_hw.h"
#include "ti_msp_dl_config.h"

#define GRAYSCALE_HW_I2C_WAIT_CYCLES  (50000u)

#define GRAYSCALE_HW_I2C_INTERRUPTS   \
    (DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | \
     DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | \
     DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER | \
     DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER | \
     DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_FULL | \
     DL_I2C_INTERRUPT_CONTROLLER_NACK | \
     DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

static uint32 grayscale_hw_error_count;

static uint8 grayscale_hw_wait_for_idle(void)
{
    uint32 wait_cycles = GRAYSCALE_HW_I2C_WAIT_CYCLES;

    while ((0u != (DL_I2C_getControllerStatus(I2C_OLED_INST) &
                    DL_I2C_CONTROLLER_STATUS_BUSY)) &&
           (0u != wait_cycles))
    {
        wait_cycles--;
    }

    return ((0u != wait_cycles) &&
            (0u == (DL_I2C_getControllerStatus(I2C_OLED_INST) &
                     DL_I2C_CONTROLLER_STATUS_ERROR))) ? 1u : 0u;
}

void grayscale_hw_init(void)
{
    grayscale_hw_error_count = 0u;
}

uint8 grayscale_hw_read_states(uint8 values[GRAYSCALE_HW_CHANNELS])
{
    uint8 register_address = GRAYSCALE_HW_STATE_REGISTER;
    uint8 packed_states;
    uint8 index;

    if ((NULL == values) || (0u != oled_hw_is_busy()))
    {
        return 0u;
    }

    /* The OLED owns I2C0 asynchronously, so keep this short poll atomic. */
    NVIC_DisableIRQ(I2C_OLED_INST_INT_IRQN);
    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    DL_I2C_clearInterruptStatus(I2C_OLED_INST,
        GRAYSCALE_HW_I2C_INTERRUPTS);
    (void)DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, &register_address, 1u);
    DL_I2C_startControllerTransfer(I2C_OLED_INST,
        GRAYSCALE_HW_I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX, 1u);

    if (0u == grayscale_hw_wait_for_idle())
    {
        goto error;
    }

    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    DL_I2C_clearInterruptStatus(I2C_OLED_INST,
        GRAYSCALE_HW_I2C_INTERRUPTS);
    DL_I2C_startControllerTransfer(I2C_OLED_INST,
        GRAYSCALE_HW_I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX, 1u);

    if ((0u == grayscale_hw_wait_for_idle()) ||
        (0u != DL_I2C_isControllerRXFIFOEmpty(I2C_OLED_INST)))
    {
        goto error;
    }

    packed_states = DL_I2C_receiveControllerData(I2C_OLED_INST);
    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    DL_I2C_clearInterruptStatus(I2C_OLED_INST,
        GRAYSCALE_HW_I2C_INTERRUPTS);
    NVIC_EnableIRQ(I2C_OLED_INST_INT_IRQN);

    for (index = 0u; index < GRAYSCALE_HW_CHANNELS; index++)
    {
        values[index] = (packed_states >> index) & 0x01u;
    }
    return 1u;

error:
    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    DL_I2C_clearInterruptStatus(I2C_OLED_INST,
        GRAYSCALE_HW_I2C_INTERRUPTS);
    NVIC_EnableIRQ(I2C_OLED_INST_INT_IRQN);
    grayscale_hw_error_count++;
    return 0u;
}

uint32 grayscale_hw_get_error_count(void)
{
    return grayscale_hw_error_count;
}
