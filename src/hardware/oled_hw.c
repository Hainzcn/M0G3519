#include "oled_hw.h"

#include "heartbeat.h"
#include "ti_msp_dl_config.h"

#include "string.h"

#define OLED_HW_I2C_CTRL_CMD          (0x00u)
#define OLED_HW_I2C_CTRL_DATA         (0x40u)
#define OLED_HW_TX_MAX_PAYLOAD        (128u)
#define OLED_HW_TX_BUFFER_SIZE        (OLED_HW_TX_MAX_PAYLOAD + 1u)
#define OLED_HW_TRANSFER_TIMEOUT_MS   (20u)

#define OLED_HW_I2C_INTERRUPTS        \
    (DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | \
     DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER | \
     DL_I2C_INTERRUPT_CONTROLLER_NACK | \
     DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

typedef enum
{
    OLED_HW_STATE_OFFLINE = 0,
    OLED_HW_STATE_INIT,
    OLED_HW_STATE_IDLE,
    OLED_HW_STATE_TX,
    OLED_HW_STATE_TX_DONE,
    OLED_HW_STATE_ERROR,
} oled_hw_state_enum;

static uint8 oled_hw_tx_buffer[OLED_HW_TX_BUFFER_SIZE];
static volatile uint16 oled_hw_tx_length;
static volatile uint16 oled_hw_tx_sent;
static volatile oled_hw_state_enum oled_hw_state;
static uint32 oled_hw_deadline_ms;
static uint32 oled_hw_error_count;

static const uint8 oled_hw_init_table[] =
{
    0xAE,
    0xD5, 0x80,
    0xA8, 0x3F,
    0xD3, 0x00,
    0x40,
    0x8D, 0x14,
    0x20, 0x00,
    0xA1,
    0xC8,
    0xDA, 0x12,
    0x81, 0xCF,
    0xD9, 0xF1,
    0xDB, 0x40,
    0xA4,
    0xA6,
    0xAF,
};

static void oled_hw_abort(void)
{
    DL_I2C_disableInterrupt(I2C_OLED_INST, OLED_HW_I2C_INTERRUPTS);
    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    oled_hw_state = OLED_HW_STATE_ERROR;
    oled_hw_error_count++;
}

static uint8 oled_hw_start_transfer(const uint8 *data, uint16 len,
                                    uint8 control,
                                    oled_hw_state_enum transfer_state)
{
    uint16 initial_count;

    if (((NULL == data) && (0u != len)) ||
        (len > OLED_HW_TX_MAX_PAYLOAD) ||
        ((OLED_HW_STATE_IDLE != oled_hw_state) &&
         (OLED_HW_STATE_OFFLINE != oled_hw_state)))
    {
        return 0u;
    }

    if (0u != len)
    {
        memcpy(&oled_hw_tx_buffer[1], data, len);
    }
    oled_hw_tx_buffer[0] = control;
    oled_hw_tx_length = (uint16)(len + 1u);

    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    initial_count = DL_I2C_fillControllerTXFIFO(
        I2C_OLED_INST, oled_hw_tx_buffer, oled_hw_tx_length);
    oled_hw_tx_sent = initial_count;
    oled_hw_state = transfer_state;
    oled_hw_deadline_ms = heartbeat_get_ms() + OLED_HW_TRANSFER_TIMEOUT_MS;

    DL_I2C_clearInterruptStatus(I2C_OLED_INST, OLED_HW_I2C_INTERRUPTS);
    DL_I2C_enableInterrupt(I2C_OLED_INST, OLED_HW_I2C_INTERRUPTS);
    if (initial_count >= oled_hw_tx_length)
    {
        DL_I2C_disableInterrupt(I2C_OLED_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    }

    DL_I2C_startControllerTransfer(I2C_OLED_INST, OLED_HW_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, oled_hw_tx_length);
    return 1u;
}

void oled_hw_init(void)
{
    DL_I2C_disableInterrupt(I2C_OLED_INST, OLED_HW_I2C_INTERRUPTS);
    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    NVIC_DisableIRQ(I2C_OLED_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(I2C_OLED_INST_INT_IRQN);
    NVIC_SetPriority(I2C_OLED_INST_INT_IRQN, 3u);
    NVIC_EnableIRQ(I2C_OLED_INST_INT_IRQN);

    oled_hw_state = OLED_HW_STATE_OFFLINE;
    (void)oled_hw_start_transfer(oled_hw_init_table,
        (uint16)sizeof(oled_hw_init_table), OLED_HW_I2C_CTRL_CMD,
        OLED_HW_STATE_INIT);
}

void oled_hw_process(void)
{
    oled_hw_state_enum state = oled_hw_state;

    if (OLED_HW_STATE_TX_DONE == state)
    {
        oled_hw_state = OLED_HW_STATE_IDLE;
        return;
    }

    if ((OLED_HW_STATE_INIT == state) || (OLED_HW_STATE_TX == state))
    {
        if ((int32)(heartbeat_get_ms() - oled_hw_deadline_ms) >= 0)
        {
            oled_hw_abort();
        }
    }
}

uint8 oled_hw_is_ready(void)
{
    oled_hw_state_enum state = oled_hw_state;
    return ((OLED_HW_STATE_IDLE == state) ||
            (OLED_HW_STATE_TX == state) ||
            (OLED_HW_STATE_TX_DONE == state)) ? 1u : 0u;
}

uint8 oled_hw_is_busy(void)
{
    oled_hw_state_enum state = oled_hw_state;
    return ((OLED_HW_STATE_INIT == state) ||
            (OLED_HW_STATE_TX == state)) ? 1u : 0u;
}

uint8 oled_hw_write_cmd(uint8 cmd)
{
    return oled_hw_write_cmds(&cmd, 1u);
}

uint8 oled_hw_write_cmds(const uint8 *commands, uint8 len)
{
    return oled_hw_start_transfer(commands, len, OLED_HW_I2C_CTRL_CMD,
                                  OLED_HW_STATE_TX);
}

uint8 oled_hw_write_data(const uint8 *data, uint32 len)
{
    if (len > OLED_HW_TX_MAX_PAYLOAD)
    {
        return 0u;
    }

    return oled_hw_start_transfer(data, (uint16)len, OLED_HW_I2C_CTRL_DATA,
                                  OLED_HW_STATE_TX);
}

uint32 oled_hw_get_error_count(void)
{
    return oled_hw_error_count;
}

void I2C0_IRQHandler(void)
{
    switch (DL_I2C_getPendingInterrupt(I2C_OLED_INST))
    {
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            if (oled_hw_tx_sent < oled_hw_tx_length)
            {
                oled_hw_tx_sent += DL_I2C_fillControllerTXFIFO(
                    I2C_OLED_INST, &oled_hw_tx_buffer[oled_hw_tx_sent],
                    (uint16)(oled_hw_tx_length - oled_hw_tx_sent));
            }
            if (oled_hw_tx_sent >= oled_hw_tx_length)
            {
                DL_I2C_disableInterrupt(I2C_OLED_INST,
                    DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            DL_I2C_disableInterrupt(I2C_OLED_INST, OLED_HW_I2C_INTERRUPTS);
            oled_hw_state = OLED_HW_STATE_TX_DONE;
            break;

        case DL_I2C_IIDX_CONTROLLER_NACK:
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
            oled_hw_abort();
            break;

        default:
            break;
    }
}
