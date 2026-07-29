#ifndef OLED_HW_H_
#define OLED_HW_H_

#include "zf_common_typedef.h"

/*
 * GME12864-49（0.96 寸 128×64，4 针 I2C，SSD1306/兼容）硬件层。
 *
 * 使用 SysConfig 配置的硬件 I2C0（400 kHz）：
 *   SCL = B0 (PB0, I2C0_SCL)
 *   SDA = B1 (PB1, I2C0_SDA)
 *
 * 注：PB12/PB13 无 I2C 复用，硬件 I2C 须接 B0/B1。
 *
 * VCC = 3.3V；GND 共地。
 * I2C 7 位地址默认 0x3C；SA0 接高时改为 0x3D。
 */

#define OLED_HW_WIDTH               (128)
#define OLED_HW_HEIGHT              (64)
#define OLED_HW_PAGE_COUNT          (OLED_HW_HEIGHT / 8)

#define OLED_HW_I2C_ADDR            (0x3C)

void  oled_hw_init(void);
void  oled_hw_process(void);
uint8 oled_hw_is_ready(void);
uint8 oled_hw_is_busy(void);
uint8 oled_hw_write_cmd(uint8 cmd);
uint8 oled_hw_write_cmds(const uint8 *commands, uint8 len);
uint8 oled_hw_write_data(const uint8 *data, uint32 len);
uint32 oled_hw_get_error_count(void);

#endif
