#ifndef OLED_H_
#define OLED_H_

#include "oled_hw.h"

#define OLED_FONT_6X8               (0)
#define OLED_FONT_8X16              (1)

void  oled_init(void);
uint8 oled_is_ready(void);
void  oled_clear(void);
void  oled_set_pixel(uint8 x, uint8 y, uint8 on);
void  oled_show_char(uint8 x, uint8 page, char chr, uint8 font);
void  oled_show_string(uint8 x, uint8 page, const char *text, uint8 font);
void  oled_show_uint(uint8 x, uint8 page, uint32 value, uint8 font);
void  oled_show_int(uint8 x, uint8 page, int32 value, uint8 font);
void  oled_refresh(void);

#endif
