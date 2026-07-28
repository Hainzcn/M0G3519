#include "oled.h"

#include "string.h"
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
    memset(oled_buffer, 0, OLED_BUFFER_SIZE);
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
        uint16 base = (uint16)page * OLED_HW_WIDTH;

        for (col = 0; col < 6; col ++)
        {
            if ((uint16)x + col >= OLED_HW_WIDTH)
            {
                break;
            }

            oled_buffer[base + x + col] = ascii_font_6x8[index][col];
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

void oled_clear_area(uint8 x, uint8 y, uint8 w, uint8 h)
{
    uint8 dx;
    uint8 dy;

    for (dy = 0; dy < h; dy ++)
    {
        for (dx = 0; dx < w; dx ++)
        {
            oled_set_pixel((uint8)(x + dx), (uint8)(y + dy), 0);
        }
    }
}

void oled_clear_page(uint8 page)
{
    if (page >= OLED_HW_PAGE_COUNT)
    {
        return;
    }

    memset(oled_buffer + ((uint32)page * OLED_HW_WIDTH), 0, OLED_HW_WIDTH);
}

void oled_clear_page_segment(uint8 page, uint8 x, uint8 w)
{
    if (page >= OLED_HW_PAGE_COUNT)
    {
        return;
    }

    if ((uint16)x + w > OLED_HW_WIDTH)
    {
        if (x >= OLED_HW_WIDTH)
        {
            return;
        }

        w = (uint8)(OLED_HW_WIDTH - x);
    }

    memset(oled_buffer + ((uint32)page * OLED_HW_WIDTH) + x, 0, w);
}

void oled_fill_page_bar(uint8 page, const uint8 *values, uint8 count, uint8 block_width)
{
    uint8  ch;
    uint8 *row;

    if ((page >= OLED_HW_PAGE_COUNT) || (NULL == values) || (0u == count) || (0u == block_width))
    {
        return;
    }

    row = oled_buffer + ((uint32)page * OLED_HW_WIDTH);
    for (ch = 0; ch < count; ch ++)
    {
        memset(row + ((uint32)ch * block_width),
               values[ch] ? 0xFFu : 0x00u,
               block_width);
    }
}

static void oled_refresh_page_range(uint8 page_start, uint8 page_end)
{
    if (0 == oled_hw_is_ready())
    {
        return;
    }

    if (page_start > page_end)
    {
        return;
    }

    if (page_end >= OLED_HW_PAGE_COUNT)
    {
        return;
    }

    oled_hw_write_cmd(0x20);
    oled_hw_write_cmd(0x00);
    oled_hw_write_cmd(0x21);
    oled_hw_write_cmd(0x00);
    oled_hw_write_cmd(0x7F);
    oled_hw_write_cmd(0x22);
    oled_hw_write_cmd(page_start);
    oled_hw_write_cmd(page_end);
    oled_hw_write_data(oled_buffer + ((uint32)page_start * OLED_HW_WIDTH),
                       (uint32)(page_end - page_start + 1u) * OLED_HW_WIDTH);
}

void oled_refresh(void)
{
    oled_refresh_page_range(0, (uint8)(OLED_HW_PAGE_COUNT - 1u));
}

void oled_refresh_pages(uint8 page_start, uint8 page_end)
{
    oled_refresh_page_range(page_start, page_end);
}
