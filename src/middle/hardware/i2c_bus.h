#ifndef I2C_BUS_H_
#define I2C_BUS_H_

#include "zf_common_typedef.h"

typedef enum
{
    I2C_BUS_CLIENT_NONE = 0,
    I2C_BUS_CLIENT_IR_TRACKING,
    I2C_BUS_CLIENT_OLED,
} i2c_bus_client_t;

typedef enum
{
    I2C_BUS_RESULT_PENDING = 0,
    I2C_BUS_RESULT_DONE,
    I2C_BUS_RESULT_ERROR,
} i2c_bus_result_t;

void i2c_bus_init(void);
void i2c_bus_process(void);
uint8 i2c_bus_start_write(i2c_bus_client_t client, uint8 address,
                          const uint8 *data, uint16 length);
uint8 i2c_bus_start_write_read(i2c_bus_client_t client, uint8 address,
                               const uint8 *write_data, uint16 write_length,
                               uint8 *read_data, uint16 read_length);
i2c_bus_result_t i2c_bus_take_result(i2c_bus_client_t client);

#endif
