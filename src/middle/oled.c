#include "oled.h"

#include "zf_common_font.h"

#define OLED_BUFFER_SIZE            (OLED_HW_WIDTH * OLED_HW_PAGE_COUNT)

static uint8 oled_buffer[OLED_BUFFER_SIZE];

static uint8 oled_font_index(char chr)
{
    if ((chr < ' ') || (chr > '~'))
    {
        chr = ' ';
    }

    return (uint8)(chr - ' ');
}

void oled_init(void)
{
    oled_hw_init();
    oled_clear();
    oled_refresh();
}

uint8 oled_is_ready(void)
{
    return oled_hw_is_ready();
}

void oled_clear(void)
{
    uint32 index;

    for (index = 0; index < OLED_BUFFER_SIZE; index ++)
    {
        oled_buffer[index] = 0;
    }
}

void oled_set_pixel(uint8 x, uint8 y, uint8 on)
{
    uint16 index;
    uint8  mask;

    if ((x >= OLED_HW_WIDTH) || (y >= OLED_HW_HEIGHT))
    {
        return;
    }

    index = (uint16)((y >> 3) * OLED_HW_WIDTH + x);
    mask  = (uint8)(1u << (y & 0x07u));

    if (on)
    {
        oled_buffer[index] |= mask;
    }
    else
    {
        oled_buffer[index] &= (uint8)(~mask);
    }
}

static void oled_draw_font_column(uint8 x, uint8 y, uint8 column_byte)
{
    uint8 row;

    for (row = 0; row < 8; row ++)
    {
        if (0 != (column_byte & (1u << row)))
        {
            oled_set_pixel(x, (uint8)(y + row), 1);
        }
    }
}

void oled_show_char(uint8 x, uint8 page, char chr, uint8 font)
{
    uint8 index;
    uint8 col;

    if (page >= OLED_HW_PAGE_COUNT)
    {
        return;
    }

    index = oled_font_index(chr);

    if (OLED_FONT_8X16 == font)
    {
        if ((page + 1) >= OLED_HW_PAGE_COUNT)
        {
            return;
        }

        for (col = 0; col < 8; col ++)
        {
            if ((uint16)x + col >= OLED_HW_WIDTH)
            {
                break;
            }

            oled_draw_font_column((uint8)(x + col), (uint8)(page * 8),
                                  ascii_font_8x16[index][col]);
            oled_draw_font_column((uint8)(x + col), (uint8)((page + 1) * 8),
                                  ascii_font_8x16[index][col + 8]);
        }
    }
    else
    {
        for (col = 0; col < 6; col ++)
        {
            if ((uint16)x + col >= OLED_HW_WIDTH)
            {
                break;
            }

            oled_draw_font_column((uint8)(x + col), (uint8)(page * 8),
                                  ascii_font_6x8[index][col]);
        }
    }
}

void oled_show_string(uint8 x, uint8 page, const char *text, uint8 font)
{
    uint8 cursor = x;
    uint8 char_width = (OLED_FONT_8X16 == font) ? 8u : 6u;

    if (NULL == text)
    {
        return;
    }

    while ((*text != '\0') && (cursor < OLED_HW_WIDTH))
    {
        oled_show_char(cursor, page, *text, font);
        cursor = (uint8)(cursor + char_width);
        text ++;
    }
}

static void oled_show_digits(uint8 x, uint8 page, uint32 value, uint8 font, uint8 leading)
{
    char buffer[11];
    uint8 index = 10;

    buffer[index] = '\0';

    if (0 == value)
    {
        buffer[-- index] = '0';
    }
    else
    {
        while ((value > 0) && (index > 0))
        {
            buffer[-- index] = (char)('0' + (value % 10));
            value /= 10;
        }
    }

    while ((leading > 0) && (index > 0))
    {
        buffer[-- index] = '0';
        leading --;
    }

    oled_show_string(x, page, buffer + index, font);
}

void oled_show_uint(uint8 x, uint8 page, uint32 value, uint8 font)
{
    oled_show_digits(x, page, value, font, 0);
}

void oled_show_int(uint8 x, uint8 page, int32 value, uint8 font)
{
    if (value < 0)
    {
        oled_show_char(x, page, '-', font);
        oled_show_uint((uint8)(x + ((OLED_FONT_8X16 == font) ? 8u : 6u)),
                       page, (uint32)(-value), font);
    }
    else
    {
        oled_show_uint(x, page, (uint32)value, font);
    }
}

void oled_refresh(void)
{
    if (0 == oled_hw_is_ready())
    {
        return;
    }

    oled_hw_write_cmd(0x20);
    oled_hw_write_cmd(0x00);
    oled_hw_write_cmd(0x21);
    oled_hw_write_cmd(0x00);
    oled_hw_write_cmd(0x7F);
    oled_hw_write_cmd(0x22);
    oled_hw_write_cmd(0x00);
    oled_hw_write_cmd(0x07);
    oled_hw_write_data(oled_buffer, OLED_BUFFER_SIZE);
}
