#include "oled_hw.h"

#include "ti_msp_dl_config.h"

#define OLED_HW_I2C_CTRL_CMD        (0x00)
#define OLED_HW_I2C_CTRL_DATA       (0x40)

static uint8  oled_hw_ready;
static uint32 oled_hw_delay_cycles;

static void oled_hw_calc_delay_cycles(void)
{
    DL_I2C_ClockConfig clock_config;
    uint32             i2c_clock_hz = 80000000u;

    DL_I2C_getClockConfig(I2C_OLED_INST, &clock_config);

    switch (clock_config.clockSel)
    {
        case DL_I2C_CLOCK_BUSCLK:
            i2c_clock_hz = CPUCLK_FREQ;
            break;
        case DL_I2C_CLOCK_MFCLK:
            i2c_clock_hz = 4000000u;
            break;
        default:
            break;
    }

    oled_hw_delay_cycles = (3u * (clock_config.divideRatio + 1u)) *
                           (CPUCLK_FREQ / i2c_clock_hz);
}

static uint8 oled_hw_wait_idle(void)
{
    uint32 timeout = 1000000u;

    while ((0u == (DL_I2C_getControllerStatus(I2C_OLED_INST) &
                   DL_I2C_CONTROLLER_STATUS_IDLE)) &&
           (timeout > 0u))
    {
        timeout --;
    }

    return (timeout > 0u) ? 1u : 0u;
}

static uint8 oled_hw_transfer(const uint8 *data, uint32 len)
{
    uint32 sent  = 0u;
    uint32 chunk = 0u;
    uint32 timeout;

    if ((NULL == data) || (0u == len))
    {
        return 0u;
    }

    if (0u == oled_hw_wait_idle())
    {
        return 0u;
    }

    chunk = DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, data, len);
    sent  = chunk;

    DL_I2C_startControllerTransfer(I2C_OLED_INST, OLED_HW_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, len);
    delay_cycles(oled_hw_delay_cycles);

    timeout = 4000000u;
    while ((sent < len) && (timeout > 0u))
    {
        if (0u != (DL_I2C_getControllerStatus(I2C_OLED_INST) &
                   DL_I2C_CONTROLLER_STATUS_ERROR))
        {
            return 0u;
        }

        if (false == DL_I2C_isControllerTXFIFOFull(I2C_OLED_INST))
        {
            chunk = DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, data + sent,
                                                len - sent);
            sent += chunk;
        }

        timeout --;
    }

    if (0u == timeout)
    {
        return 0u;
    }

    timeout = 4000000u;
    while ((0u != (DL_I2C_getControllerStatus(I2C_OLED_INST) &
                    DL_I2C_CONTROLLER_STATUS_BUSY)) &&
           (timeout > 0u))
    {
        timeout --;
    }

    if (0u == timeout)
    {
        return 0u;
    }

    if (0u != (DL_I2C_getControllerStatus(I2C_OLED_INST) &
               DL_I2C_CONTROLLER_STATUS_ERROR))
    {
        return 0u;
    }

    return (sent >= len) ? 1u : 0u;
}

static uint8 oled_hw_write_payload(const uint8 *data, uint32 len, uint8 ctrl)
{
    uint8  frame[9];
    uint32 payload_len;
    uint32 chunk_len;
    uint32 sent = 0u;
    uint32 chunk;
    uint32 timeout;

    if ((NULL == data) && (0u != len))
    {
        return 0u;
    }

    if (0u == len)
    {
        frame[0] = ctrl;
        return oled_hw_transfer(frame, 1u);
    }

    if (0u == oled_hw_wait_idle())
    {
        return 0u;
    }

    payload_len = len + 1u;
    frame[0]    = ctrl;
    chunk_len   = (len > 7u) ? 7u : len;

    {
        uint32 index;

        for (index = 0; index < chunk_len; index ++)
        {
            frame[index + 1u] = data[index];
        }
    }

    chunk = DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, frame, chunk_len + 1u);
    sent  = (chunk > 0u) ? (chunk - 1u) : 0u;

    DL_I2C_startControllerTransfer(I2C_OLED_INST, OLED_HW_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, payload_len);
    delay_cycles(oled_hw_delay_cycles);

    timeout = 8000000u;
    while ((sent < len) && (timeout > 0u))
    {
        if (0u != (DL_I2C_getControllerStatus(I2C_OLED_INST) &
                   DL_I2C_CONTROLLER_STATUS_ERROR))
        {
            return 0u;
        }

        if (false == DL_I2C_isControllerTXFIFOFull(I2C_OLED_INST))
        {
            chunk = DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, data + sent,
                                                len - sent);
            sent += chunk;
        }

        timeout --;
    }

    if (0u == timeout)
    {
        return 0u;
    }

    timeout = 8000000u;
    while ((0u != (DL_I2C_getControllerStatus(I2C_OLED_INST) &
                    DL_I2C_CONTROLLER_STATUS_BUSY)) &&
           (timeout > 0u))
    {
        timeout --;
    }

    if (0u == timeout)
    {
        return 0u;
    }

    if (0u != (DL_I2C_getControllerStatus(I2C_OLED_INST) &
               DL_I2C_CONTROLLER_STATUS_ERROR))
    {
        return 0u;
    }

    return (sent >= len) ? 1u : 0u;
}

static uint8 oled_hw_send_init_table(void)
{
    static const uint8 init_table[] =
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
    uint32 index;

    for (index = 0; index < sizeof(init_table); index ++)
    {
        if (0u == oled_hw_write_cmd(init_table[index]))
        {
            return 0u;
        }
    }

    return 1u;
}

static uint8 oled_hw_probe(void)
{
    uint8 frame[2];

    frame[0] = OLED_HW_I2C_CTRL_CMD;
    frame[1] = 0xAE;

    return oled_hw_transfer(frame, sizeof(frame));
}

void oled_hw_init(void)
{
    oled_hw_ready = 0u;
    oled_hw_calc_delay_cycles();

    if (0u == oled_hw_probe())
    {
        return;
    }

    if (0u == oled_hw_send_init_table())
    {
        return;
    }

    oled_hw_ready = 1u;
}

uint8 oled_hw_is_ready(void)
{
    return oled_hw_ready;
}

uint8 oled_hw_write_cmd(uint8 cmd)
{
    uint8 frame[2];

    frame[0] = OLED_HW_I2C_CTRL_CMD;
    frame[1] = cmd;
    if (0u == oled_hw_transfer(frame, sizeof(frame)))
    {
        oled_hw_ready = 0u;
        return 0u;
    }

    return 1u;
}

void oled_hw_write_data(const uint8 *data, uint32 len)
{
    if (0u == oled_hw_write_payload(data, len, OLED_HW_I2C_CTRL_DATA))
    {
        oled_hw_ready = 0u;
    }
}
