#include "oled_app.h"

#include "encoder.h"
#include "grayscale.h"
#include "heartbeat.h"
#include "imu.h"
#include "oled.h"

#define OLED_APP_REFRESH_PERIOD_MS      (500)
#define OLED_APP_GS_BAR_Y               (56)
#define OLED_APP_GS_BAR_H               (8)
#define OLED_APP_GS_BLOCK_W             (OLED_HW_WIDTH / GRAYSCALE_CHANNELS)

static uint32 oled_app_last_refresh_ms;

static void oled_app_draw_gs_bar(const uint8 *values)
{
    uint8 ch;
    uint8 x0;
    uint8 x;
    uint8 y;

    for (ch = 0; ch < GRAYSCALE_CHANNELS; ch ++)
    {
        if (0u == values[ch])
        {
            continue;
        }

        x0 = (uint8)(ch * OLED_APP_GS_BLOCK_W);
        for (x = x0; x < (x0 + OLED_APP_GS_BLOCK_W); x ++)
        {
            for (y = OLED_APP_GS_BAR_Y; y < (OLED_APP_GS_BAR_Y + OLED_APP_GS_BAR_H); y ++)
            {
                oled_set_pixel(x, y, 1);
            }
        }
    }
}

void oled_app_init(void)
{
    oled_init();
    oled_app_last_refresh_ms = heartbeat_get_ms();
}

void oled_app_process(void)
{
    const imu_angle_t *angle;
    const uint8       *gs;
    uint32 now_ms;

    if (0 == oled_is_ready())
    {
        return;
    }

    now_ms = heartbeat_get_ms();
    if ((now_ms - oled_app_last_refresh_ms) < OLED_APP_REFRESH_PERIOD_MS)
    {
        return;
    }

    oled_app_last_refresh_ms = now_ms;

    oled_clear();
    oled_show_string(0, 0, "MSPM0G3519", OLED_FONT_6X8);

    angle = imu_get_angle();
    oled_show_string(0, 1, "Yaw:", OLED_FONT_6X8);
    oled_show_int(30, 1, (int32)angle->yaw, OLED_FONT_6X8);

    oled_show_string(0, 3, "L:", OLED_FONT_6X8);
    oled_show_int(12, 3, encoder_get_left_rpm(), OLED_FONT_6X8);
    oled_show_string(64, 3, "R:", OLED_FONT_6X8);
    oled_show_int(76, 3, encoder_get_right_rpm(), OLED_FONT_6X8);

    gs = grayscale_get_values();
    oled_app_draw_gs_bar(gs);
    oled_refresh();
}
