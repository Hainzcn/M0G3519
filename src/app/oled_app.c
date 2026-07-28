#include "oled_app.h"

#include "encoder.h"
#include "grayscale.h"
#include "heartbeat.h"
#include "imu.h"
#include "oled.h"

#include "string.h"

/*
 * I2C 400 kHz 下整屏 1024 B 推送约 25 ms，单页约 3 ms。
 * 循迹条按页字节直写帧缓冲（非逐像素、非字模），数值区 6x8 字模亦直写页缓冲；
 * 仅在数据变化时推送 I2C，避免无效刷新。
 */
#define OLED_APP_GS_PERIOD_MS           (50)
#define OLED_APP_TEXT_PERIOD_MS         (100)

#define OLED_APP_PAGE_TITLE             (0)
#define OLED_APP_PAGE_YAW               (1)
#define OLED_APP_PAGE_RPM               (3)
#define OLED_APP_PAGE_GS                (7)

#define OLED_APP_GS_BLOCK_W             (OLED_HW_WIDTH / GRAYSCALE_CHANNELS)

#define OLED_APP_YAW_VALUE_X            (30)
#define OLED_APP_LEFT_RPM_VALUE_X       (12)
#define OLED_APP_RIGHT_RPM_VALUE_X      (76)
#define OLED_APP_RPM_VALUE_W            (42)

static uint32 oled_app_last_gs_ms;
static uint32 oled_app_last_text_ms;

static uint8  oled_app_gs_cache[GRAYSCALE_CHANNELS];
static int32  oled_app_yaw_cache;
static int32  oled_app_left_rpm_cache;
static int32  oled_app_right_rpm_cache;

static void oled_app_draw_static_labels(void)
{
    oled_show_string(0, OLED_APP_PAGE_TITLE, "MSPM0G3519", OLED_FONT_6X8);
    oled_show_string(0, OLED_APP_PAGE_YAW, "Yaw:", OLED_FONT_6X8);
    oled_show_string(0, OLED_APP_PAGE_RPM, "L:", OLED_FONT_6X8);
    oled_show_string(64, OLED_APP_PAGE_RPM, "R:", OLED_FONT_6X8);
}

static uint8 oled_app_render_gs(const uint8 *values)
{
    if (0 == memcmp(values, oled_app_gs_cache, GRAYSCALE_CHANNELS))
    {
        return 0;
    }

    memcpy(oled_app_gs_cache, values, GRAYSCALE_CHANNELS);
    oled_fill_page_bar(OLED_APP_PAGE_GS, values, GRAYSCALE_CHANNELS, OLED_APP_GS_BLOCK_W);
    return 1;
}

static uint8 oled_app_render_text(void)
{
    const imu_angle_t *angle;
    int32 yaw;
    int32 left_rpm;
    int32 right_rpm;
    uint8 yaw_ready;

    angle     = imu_get_angle();
    yaw_ready = imu_is_type_ready(IMU_FLAG_ANGLE);
    yaw       = yaw_ready ? (int32)angle->yaw : 0;
    left_rpm  = encoder_get_left_rpm();
    right_rpm = encoder_get_right_rpm();

    if (yaw_ready &&
        (yaw == oled_app_yaw_cache) &&
        (left_rpm == oled_app_left_rpm_cache) &&
        (right_rpm == oled_app_right_rpm_cache))
    {
        return 0;
    }

    if (!yaw_ready &&
        (0x7FFFFFFEL == oled_app_yaw_cache) &&
        (left_rpm == oled_app_left_rpm_cache) &&
        (right_rpm == oled_app_right_rpm_cache))
    {
        return 0;
    }

    oled_app_yaw_cache       = yaw_ready ? yaw : 0x7FFFFFFEL;
    oled_app_left_rpm_cache  = left_rpm;
    oled_app_right_rpm_cache = right_rpm;

    oled_clear_page_segment(OLED_APP_PAGE_YAW, OLED_APP_YAW_VALUE_X,
                            (uint8)(OLED_HW_WIDTH - OLED_APP_YAW_VALUE_X));
    oled_clear_page_segment(OLED_APP_PAGE_RPM, OLED_APP_LEFT_RPM_VALUE_X,
                            OLED_APP_RPM_VALUE_W);
    oled_clear_page_segment(OLED_APP_PAGE_RPM, OLED_APP_RIGHT_RPM_VALUE_X,
                            OLED_APP_RPM_VALUE_W);

    if (yaw_ready)
    {
        oled_show_int(OLED_APP_YAW_VALUE_X, OLED_APP_PAGE_YAW, yaw, OLED_FONT_6X8);
    }
    else
    {
        oled_show_string(OLED_APP_YAW_VALUE_X, OLED_APP_PAGE_YAW, "---", OLED_FONT_6X8);
    }

    oled_show_int(OLED_APP_LEFT_RPM_VALUE_X, OLED_APP_PAGE_RPM, left_rpm, OLED_FONT_6X8);
    oled_show_int(OLED_APP_RIGHT_RPM_VALUE_X, OLED_APP_PAGE_RPM, right_rpm, OLED_FONT_6X8);
    return 1;
}

void oled_app_init(void)
{
    const uint8 *gs_values;
    uint32       now_ms;

    oled_hw_init();
    oled_clear();

    now_ms = heartbeat_get_ms();
    oled_app_last_gs_ms   = now_ms;
    oled_app_last_text_ms = now_ms;

    memset(oled_app_gs_cache, 0xFF, GRAYSCALE_CHANNELS);
    oled_app_yaw_cache       = 0x7FFFFFFF;
    oled_app_left_rpm_cache  = 0x7FFFFFFF;
    oled_app_right_rpm_cache = 0x7FFFFFFF;

    oled_app_draw_static_labels();
    oled_app_render_text();
    gs_values = grayscale_get_values();
    oled_app_render_gs(gs_values);
    oled_refresh();
}

void oled_app_process(void)
{
    uint32 now_ms;

    if (0 == oled_is_ready())
    {
        return;
    }

    now_ms = heartbeat_get_ms();

    if ((now_ms - oled_app_last_text_ms) >= OLED_APP_TEXT_PERIOD_MS)
    {
        oled_app_last_text_ms = now_ms;
        if (oled_app_render_text())
        {
            oled_refresh_pages(OLED_APP_PAGE_YAW, OLED_APP_PAGE_RPM);
        }
    }

    if ((now_ms - oled_app_last_gs_ms) >= OLED_APP_GS_PERIOD_MS)
    {
        oled_app_last_gs_ms = now_ms;
        if (oled_app_render_gs(grayscale_get_values()))
        {
            oled_refresh_pages(OLED_APP_PAGE_GS, OLED_APP_PAGE_GS);
        }
    }
}
