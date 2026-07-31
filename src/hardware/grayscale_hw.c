#include "grayscale_hw.h"

#include "i2c_bus.h"

static uint8 grayscale_hw_register = GRAYSCALE_HW_DATA_REGISTER;
static uint8 grayscale_hw_sensor_bits;
static uint32 grayscale_hw_error_count;

void grayscale_hw_init(void)
{
    i2c_bus_init();
    grayscale_hw_sensor_bits = 0u;
    grayscale_hw_error_count = 0u;
}

void grayscale_hw_process(void)
{
    i2c_bus_process();
}

uint8 grayscale_hw_start_read(void)
{
    return i2c_bus_start_write_read(I2C_BUS_CLIENT_IR_TRACKING,
        GRAYSCALE_HW_I2C_ADDR, &grayscale_hw_register, 1u,
        &grayscale_hw_sensor_bits, 1u);
}

uint8 grayscale_hw_take_read(uint8 *sensor_bits)
{
    i2c_bus_result_t result =
        i2c_bus_take_result(I2C_BUS_CLIENT_IR_TRACKING);

    if (I2C_BUS_RESULT_ERROR == result)
    {
        grayscale_hw_error_count++;
        return 2u;
    }
    if (I2C_BUS_RESULT_DONE != result)
    {
        return 0u;
    }

    *sensor_bits = grayscale_hw_sensor_bits;
    return 1u;
}

uint32 grayscale_hw_get_error_count(void)
{
    return grayscale_hw_error_count;
}
