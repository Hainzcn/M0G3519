#include "oled_hw.h"

#include "i2c_bus.h"

#include "string.h"

#define OLED_HW_I2C_CTRL_CMD          (0x00u)
#define OLED_HW_I2C_CTRL_DATA         (0x40u)
#define OLED_HW_TX_MAX_PAYLOAD        (128u)
#define OLED_HW_TX_BUFFER_SIZE        (OLED_HW_TX_MAX_PAYLOAD + 1u)

typedef enum
{
    OLED_HW_STATE_OFFLINE = 0,
    OLED_HW_STATE_INIT,
    OLED_HW_STATE_IDLE,
    OLED_HW_STATE_TX,
    OLED_HW_STATE_ERROR,
} oled_hw_state_enum;

static uint8 oled_hw_tx_buffer[OLED_HW_TX_BUFFER_SIZE];
static oled_hw_state_enum oled_hw_state;
static uint32 oled_hw_error_count;

static uint8 oled_hw_start_transfer(const uint8 *data, uint16 len,
                                    uint8 control,
                                    oled_hw_state_enum transfer_state)
{
    if (((NULL == data) && (0u != len)) ||
        (len > OLED_HW_TX_MAX_PAYLOAD) ||
        ((OLED_HW_STATE_IDLE != oled_hw_state) &&
         (OLED_HW_STATE_OFFLINE != oled_hw_state)))
    {
        return 0u;
    }

    oled_hw_tx_buffer[0] = control;
    if (0u != len)
    {
        memcpy(&oled_hw_tx_buffer[1], data, len);
    }
    if (0u == i2c_bus_start_write(I2C_BUS_CLIENT_OLED, OLED_HW_I2C_ADDR,
        oled_hw_tx_buffer, (uint16)(len + 1u)))
    {
        return 0u;
    }

    oled_hw_state = transfer_state;
    return 1u;
}

void oled_hw_init(void)
{
    static const uint8 init_table[] =
    {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF,
    };

    i2c_bus_init();
    oled_hw_state = OLED_HW_STATE_OFFLINE;
    (void)oled_hw_start_transfer(init_table, (uint16)sizeof(init_table),
        OLED_HW_I2C_CTRL_CMD, OLED_HW_STATE_INIT);
}

void oled_hw_process(void)
{
    i2c_bus_result_t result;

    i2c_bus_process();
    if ((OLED_HW_STATE_INIT != oled_hw_state) &&
        (OLED_HW_STATE_TX != oled_hw_state))
    {
        return;
    }

    result = i2c_bus_take_result(I2C_BUS_CLIENT_OLED);
    if (I2C_BUS_RESULT_DONE == result)
    {
        oled_hw_state = OLED_HW_STATE_IDLE;
    }
    else if (I2C_BUS_RESULT_ERROR == result)
    {
        oled_hw_state = OLED_HW_STATE_ERROR;
        oled_hw_error_count++;
    }
}

uint8 oled_hw_is_ready(void)
{
    return ((OLED_HW_STATE_IDLE == oled_hw_state) ||
            (OLED_HW_STATE_TX == oled_hw_state)) ? 1u : 0u;
}

uint8 oled_hw_is_busy(void)
{
    return ((OLED_HW_STATE_INIT == oled_hw_state) ||
            (OLED_HW_STATE_TX == oled_hw_state)) ? 1u : 0u;
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
    return oled_hw_start_transfer(data, (uint16)len,
                                  OLED_HW_I2C_CTRL_DATA, OLED_HW_STATE_TX);
}

uint32 oled_hw_get_error_count(void)
{
    return oled_hw_error_count;
}
