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
void  oled_clear_area(uint8 x, uint8 y, uint8 w, uint8 h);
void  oled_clear_page(uint8 page);
void  oled_clear_page_segment(uint8 page, uint8 x, uint8 w);
void  oled_fill_page_bar(uint8 page, const uint8 *values, uint8 count, uint8 block_width);
void  oled_refresh(void);
void  oled_refresh_pages(uint8 page_start, uint8 page_end);

#endif
