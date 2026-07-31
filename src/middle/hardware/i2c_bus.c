#include "i2c_bus.h"

#include "heartbeat.h"
#include "ti_msp_dl_config.h"

#define I2C_BUS_TRANSFER_TIMEOUT_MS    (10u)

#define I2C_BUS_INTERRUPTS             \
    (DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | \
     DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | \
     DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER | \
     DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER | \
     DL_I2C_INTERRUPT_CONTROLLER_NACK | \
     DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

typedef enum
{
    I2C_BUS_PHASE_IDLE = 0,
    I2C_BUS_PHASE_TX,
    I2C_BUS_PHASE_WAIT_RX_IDLE,
    I2C_BUS_PHASE_RX,
    I2C_BUS_PHASE_COMPLETE,
    I2C_BUS_PHASE_ERROR,
} i2c_bus_phase_t;

typedef struct
{
    I2C_Regs *instance;
    IRQn_Type irqn;
    i2c_bus_client_t client;
    volatile i2c_bus_phase_t phase;
    const uint8 *tx_data;
    uint8 *rx_data;
    uint16 tx_length;
    volatile uint16 tx_sent;
    uint16 rx_length;
    volatile uint16 rx_received;
    uint8 address;
    uint32 deadline_ms;
} i2c_bus_context_t;

static i2c_bus_context_t i2c_bus_oled =
{
    I2C_OLED_INST, I2C_OLED_INST_INT_IRQN, I2C_BUS_CLIENT_OLED,
    I2C_BUS_PHASE_IDLE, NULL, NULL, 0u, 0u, 0u, 0u, 0u, 0u,
};

static i2c_bus_context_t i2c_bus_ir_tracking =
{
    I2C_IR_TRACKING_INST, I2C_IR_TRACKING_INST_INT_IRQN,
    I2C_BUS_CLIENT_IR_TRACKING, I2C_BUS_PHASE_IDLE,
    NULL, NULL, 0u, 0u, 0u, 0u, 0u, 0u,
};

static uint8 i2c_bus_initialized;

static i2c_bus_context_t *i2c_bus_get_context(i2c_bus_client_t client)
{
    if (I2C_BUS_CLIENT_OLED == client)
    {
        return &i2c_bus_oled;
    }
    if (I2C_BUS_CLIENT_IR_TRACKING == client)
    {
        return &i2c_bus_ir_tracking;
    }
    return NULL;
}

static void i2c_bus_finish_error(i2c_bus_context_t *context)
{
    DL_I2C_disableInterrupt(context->instance, I2C_BUS_INTERRUPTS);
    DL_I2C_resetControllerTransfer(context->instance);
    context->phase = I2C_BUS_PHASE_ERROR;
}

static void i2c_bus_init_context(i2c_bus_context_t *context, uint32 priority)
{
    DL_I2C_disableInterrupt(context->instance, I2C_BUS_INTERRUPTS);
    DL_I2C_resetControllerTransfer(context->instance);
    NVIC_DisableIRQ(context->irqn);
    NVIC_ClearPendingIRQ(context->irqn);
    NVIC_SetPriority(context->irqn, priority);
    NVIC_EnableIRQ(context->irqn);
    context->phase = I2C_BUS_PHASE_IDLE;
}

void i2c_bus_init(void)
{
    if (0u != i2c_bus_initialized)
    {
        return;
    }

    i2c_bus_init_context(&i2c_bus_oled, 3u);
    i2c_bus_init_context(&i2c_bus_ir_tracking, 1u);
    i2c_bus_initialized = 1u;
}

static void i2c_bus_process_context(i2c_bus_context_t *context,
                                    uint32 now_ms)
{
    i2c_bus_phase_t phase = context->phase;

    if (I2C_BUS_PHASE_WAIT_RX_IDLE == phase)
    {
        if (0u != (DL_I2C_getControllerStatus(context->instance) &
                   DL_I2C_CONTROLLER_STATUS_IDLE))
        {
            NVIC_DisableIRQ(context->irqn);
            if (I2C_BUS_PHASE_WAIT_RX_IDLE == context->phase)
            {
                context->phase = I2C_BUS_PHASE_RX;
                DL_I2C_startControllerTransfer(context->instance,
                    context->address, DL_I2C_CONTROLLER_DIRECTION_RX,
                    context->rx_length);
            }
            NVIC_EnableIRQ(context->irqn);
            return;
        }
    }

    if (((I2C_BUS_PHASE_TX != phase) &&
         (I2C_BUS_PHASE_WAIT_RX_IDLE != phase) &&
         (I2C_BUS_PHASE_RX != phase)) ||
        ((int32)(now_ms - context->deadline_ms) < 0))
    {
        return;
    }

    NVIC_DisableIRQ(context->irqn);
    if ((I2C_BUS_PHASE_TX == context->phase) ||
        (I2C_BUS_PHASE_WAIT_RX_IDLE == context->phase) ||
        (I2C_BUS_PHASE_RX == context->phase))
    {
        i2c_bus_finish_error(context);
    }
    NVIC_EnableIRQ(context->irqn);
}

void i2c_bus_process(void)
{
    uint32 now_ms = heartbeat_get_ms();

    i2c_bus_process_context(&i2c_bus_oled, now_ms);
    i2c_bus_process_context(&i2c_bus_ir_tracking, now_ms);
}

uint8 i2c_bus_start_write_read(i2c_bus_client_t client, uint8 address,
                               const uint8 *write_data, uint16 write_length,
                               uint8 *read_data, uint16 read_length)
{
    i2c_bus_context_t *context = i2c_bus_get_context(client);
    uint16 initial_count;

    if ((NULL == context) || (I2C_BUS_PHASE_IDLE != context->phase) ||
        (NULL == write_data) || (0u == write_length) ||
        ((0u != read_length) && (NULL == read_data)))
    {
        return 0u;
    }
    if (0u == (DL_I2C_getControllerStatus(context->instance) &
               DL_I2C_CONTROLLER_STATUS_IDLE))
    {
        return 0u;
    }

    DL_I2C_resetControllerTransfer(context->instance);
    DL_I2C_flushControllerRXFIFO(context->instance);
    initial_count = DL_I2C_fillControllerTXFIFO(
        context->instance, write_data, write_length);

    context->address = address;
    context->tx_data = write_data;
    context->tx_length = write_length;
    context->tx_sent = initial_count;
    context->rx_data = read_data;
    context->rx_length = read_length;
    context->rx_received = 0u;
    context->deadline_ms = heartbeat_get_ms() + I2C_BUS_TRANSFER_TIMEOUT_MS;
    context->phase = I2C_BUS_PHASE_TX;

    DL_I2C_clearInterruptStatus(context->instance, I2C_BUS_INTERRUPTS);
    DL_I2C_enableInterrupt(context->instance, I2C_BUS_INTERRUPTS);
    if (initial_count >= write_length)
    {
        DL_I2C_disableInterrupt(context->instance,
            DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    }
    DL_I2C_startControllerTransfer(context->instance, address,
        DL_I2C_CONTROLLER_DIRECTION_TX, write_length);
    return 1u;
}

uint8 i2c_bus_start_write(i2c_bus_client_t client, uint8 address,
                          const uint8 *data, uint16 length)
{
    return i2c_bus_start_write_read(client, address, data, length, NULL, 0u);
}

i2c_bus_result_t i2c_bus_take_result(i2c_bus_client_t client)
{
    i2c_bus_context_t *context = i2c_bus_get_context(client);
    i2c_bus_phase_t phase;

    if (NULL == context)
    {
        return I2C_BUS_RESULT_ERROR;
    }

    phase = context->phase;
    if ((I2C_BUS_PHASE_COMPLETE != phase) &&
        (I2C_BUS_PHASE_ERROR != phase))
    {
        return I2C_BUS_RESULT_PENDING;
    }

    context->phase = I2C_BUS_PHASE_IDLE;
    return (I2C_BUS_PHASE_COMPLETE == phase) ?
        I2C_BUS_RESULT_DONE : I2C_BUS_RESULT_ERROR;
}

static void i2c_bus_irq_handler(i2c_bus_context_t *context)
{
    uint16 count;

    switch (DL_I2C_getPendingInterrupt(context->instance))
    {
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            if (context->tx_sent < context->tx_length)
            {
                count = DL_I2C_fillControllerTXFIFO(context->instance,
                    &context->tx_data[context->tx_sent],
                    (uint16)(context->tx_length - context->tx_sent));
                context->tx_sent = (uint16)(context->tx_sent + count);
            }
            if (context->tx_sent >= context->tx_length)
            {
                DL_I2C_disableInterrupt(context->instance,
                    DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            if (0u == context->rx_length)
            {
                DL_I2C_disableInterrupt(context->instance, I2C_BUS_INTERRUPTS);
                context->phase = I2C_BUS_PHASE_COMPLETE;
                break;
            }

            context->phase = I2C_BUS_PHASE_WAIT_RX_IDLE;
            DL_I2C_disableInterrupt(context->instance,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            break;

        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            while ((context->rx_received < context->rx_length) &&
                   (false == DL_I2C_isControllerRXFIFOEmpty(context->instance)))
            {
                context->rx_data[context->rx_received++] =
                    DL_I2C_receiveControllerData(context->instance);
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            while ((context->rx_received < context->rx_length) &&
                   (false == DL_I2C_isControllerRXFIFOEmpty(context->instance)))
            {
                context->rx_data[context->rx_received++] =
                    DL_I2C_receiveControllerData(context->instance);
            }
            DL_I2C_disableInterrupt(context->instance, I2C_BUS_INTERRUPTS);
            context->phase = (context->rx_received == context->rx_length) ?
                I2C_BUS_PHASE_COMPLETE : I2C_BUS_PHASE_ERROR;
            break;

        case DL_I2C_IIDX_CONTROLLER_NACK:
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
            i2c_bus_finish_error(context);
            break;

        default:
            break;
    }
}

void I2C0_IRQHandler(void)
{
    i2c_bus_irq_handler(&i2c_bus_oled);
}

void I2C1_IRQHandler(void)
{
    i2c_bus_irq_handler(&i2c_bus_ir_tracking);
}
