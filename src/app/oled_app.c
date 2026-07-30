#include "oled_app.h"

#include "encoder.h"
#include "grayscale.h"
#include "heartbeat.h"
#include "imu.h"
#include "line_control.h"
#include "oled.h"

#include "string.h"

/*
 * I2C 400 kHz 下整屏 1024 B 推送约 25 ms，单页约 3 ms。
 * 循迹条按页字节直写帧缓冲（非逐像素、非字模），数值区 6x8 字模亦直写页缓冲；
 * 仅在数据变化时推送 I2C，避免无效刷新。
 */
#define OLED_APP_GS_PERIOD_MS           (50)
#define OLED_APP_TEXT_PERIOD_MS         (100)
#define OLED_APP_RECOVERY_PERIOD_MS     (1000)

#define OLED_APP_PAGE_TITLE             (0)
#define OLED_APP_PAGE_YAW               (1)
#define OLED_APP_PAGE_RPM               (3)
#define OLED_APP_PAGE_TURN              (5)
#define OLED_APP_PAGE_GS                (7)

#define OLED_APP_GS_BLOCK_W             (OLED_HW_WIDTH / GRAYSCALE_CHANNELS)

#define OLED_APP_YAW_VALUE_X            (30)
#define OLED_APP_LEFT_RPM_VALUE_X       (12)
#define OLED_APP_RIGHT_RPM_VALUE_X      (76)
#define OLED_APP_RPM_VALUE_W            (42)
#define OLED_APP_TURN_X                  (48)

#define OLED_APP_TEXT_DIRTY_YAW          (0x01u)
#define OLED_APP_TEXT_DIRTY_RPM          (0x02u)

static uint32 oled_app_last_gs_ms;
static uint32 oled_app_last_text_ms;
static uint32 oled_app_last_recovery_ms;
static uint8  oled_app_was_ready;

static uint8  oled_app_gs_cache[GRAYSCALE_CHANNELS];
static int32  oled_app_yaw_cache;
static int32  oled_app_left_rpm_cache;
static int32  oled_app_right_rpm_cache;
static uint8  oled_app_turn_cache;

/* 16x16 Song bitmap rows for "turning", rendered as black on white. */
static const uint16 oled_app_glyph_turn[16] =
{
    0x0808u, 0x3F7Fu, 0x1008u, 0x1410u,
    0x24FFu, 0x3F10u, 0x0420u, 0x047Fu,
    0x0701u, 0x3C22u, 0x1414u, 0x0408u,
    0x0404u, 0x0404u, 0x0000u, 0x0000u,
};

static const uint16 oled_app_glyph_curve[16] =
{
    0x3FFFu, 0x0110u, 0x0514u, 0x0912u,
    0x1111u, 0x0000u, 0x0FFCu, 0x0004u,
    0x0FFCu, 0x0800u, 0x0FFEu, 0x0002u,
    0x0014u, 0x0008u, 0x0000u, 0x0000u,
};

static void oled_app_draw_inverse_glyph(uint8 x, const uint16 glyph[16])
{
    uint8 row;
    uint8 col;

    for (row = 0u; row < 16u; row++)
    {
        for (col = 0u; col < 16u; col++)
        {
            uint16 mask = (uint16)(1u << (15u - col));
            oled_set_pixel((uint8)(x + col),
                           (uint8)(OLED_APP_PAGE_TURN * 8u + row),
                           (0u != (glyph[row] & mask)) ? 0u : 1u);
        }
    }
}

static uint8 oled_app_render_turn(void)
{
    const line_control_output_t *line = line_control_get_output();
    uint8 turning = line->right_curve_detected;

    if (turning == oled_app_turn_cache)
    {
        return 0u;
    }

    oled_app_turn_cache = turning;
    oled_clear_page(OLED_APP_PAGE_TURN);
    oled_clear_page((uint8)(OLED_APP_PAGE_TURN + 1u));
    if (0u != turning)
    {
        oled_app_draw_inverse_glyph(OLED_APP_TURN_X, oled_app_glyph_turn);
        oled_app_draw_inverse_glyph((uint8)(OLED_APP_TURN_X + 16u),
                                    oled_app_glyph_curve);
    }
    return 1u;
}

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
    int32 yaw_cache;
    uint8 dirty_pages = 0u;
    uint8 yaw_ready;

    angle     = imu_get_angle();
    yaw_ready = imu_is_type_ready(IMU_FLAG_ANGLE);
    yaw       = yaw_ready ? (int32)angle->yaw : 0;
    left_rpm  = encoder_get_left_rpm();
    right_rpm = encoder_get_right_rpm();
    yaw_cache = yaw_ready ? yaw : 0x7FFFFFFEL;

    if ((yaw_cache == oled_app_yaw_cache) &&
        (left_rpm == oled_app_left_rpm_cache) &&
        (right_rpm == oled_app_right_rpm_cache))
    {
        return 0;
    }

    if (yaw_cache != oled_app_yaw_cache)
    {
        dirty_pages |= OLED_APP_TEXT_DIRTY_YAW;
        oled_app_yaw_cache = yaw_cache;

        oled_clear_page_segment(OLED_APP_PAGE_YAW, OLED_APP_YAW_VALUE_X,
                                (uint8)(OLED_HW_WIDTH - OLED_APP_YAW_VALUE_X));
        if (yaw_ready)
        {
            oled_show_int(OLED_APP_YAW_VALUE_X, OLED_APP_PAGE_YAW, yaw,
                          OLED_FONT_6X8);
        }
        else
        {
            oled_show_string(OLED_APP_YAW_VALUE_X, OLED_APP_PAGE_YAW, "---",
                             OLED_FONT_6X8);
        }
    }

    if ((left_rpm != oled_app_left_rpm_cache) ||
        (right_rpm != oled_app_right_rpm_cache))
    {
        dirty_pages |= OLED_APP_TEXT_DIRTY_RPM;
        oled_app_left_rpm_cache  = left_rpm;
        oled_app_right_rpm_cache = right_rpm;

        oled_clear_page_segment(OLED_APP_PAGE_RPM, OLED_APP_LEFT_RPM_VALUE_X,
                                OLED_APP_RPM_VALUE_W);
        oled_clear_page_segment(OLED_APP_PAGE_RPM, OLED_APP_RIGHT_RPM_VALUE_X,
                                OLED_APP_RPM_VALUE_W);
        oled_show_int(OLED_APP_LEFT_RPM_VALUE_X, OLED_APP_PAGE_RPM, left_rpm,
                      OLED_FONT_6X8);
        oled_show_int(OLED_APP_RIGHT_RPM_VALUE_X, OLED_APP_PAGE_RPM, right_rpm,
                      OLED_FONT_6X8);
    }

    return dirty_pages;
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
    oled_app_last_recovery_ms = now_ms;
    oled_app_was_ready = 0u;

    memset(oled_app_gs_cache, 0xFF, GRAYSCALE_CHANNELS);
    oled_app_yaw_cache       = 0x7FFFFFFF;
    oled_app_left_rpm_cache  = 0x7FFFFFFF;
    oled_app_right_rpm_cache = 0x7FFFFFFF;
    oled_app_turn_cache      = 0xFFu;

    oled_app_draw_static_labels();
    oled_app_render_text();
    oled_app_render_turn();
    gs_values = grayscale_get_values();
    oled_app_render_gs(gs_values);
    oled_refresh();
}

void oled_app_process(void)
{
    uint32 now_ms;
    uint8 text_dirty_pages;

    now_ms = heartbeat_get_ms();
    oled_process();

    if (0 == oled_is_ready())
    {
        oled_app_was_ready = 0u;
        if ((now_ms - oled_app_last_recovery_ms) >=
            OLED_APP_RECOVERY_PERIOD_MS)
        {
            oled_app_last_recovery_ms = now_ms;
            oled_hw_init();
        }
        return;
    }

    if (0u == oled_app_was_ready)
    {
        oled_app_was_ready = 1u;
        /* Initialization or recovery invalidates the controller RAM. */
        oled_refresh();
    }

    if (0u != oled_app_render_turn())
    {
        oled_refresh_pages(OLED_APP_PAGE_TURN,
                           (uint8)(OLED_APP_PAGE_TURN + 1u));
    }

    if ((now_ms - oled_app_last_text_ms) >= OLED_APP_TEXT_PERIOD_MS)
    {
        oled_app_last_text_ms = now_ms;
        text_dirty_pages = oled_app_render_text();
        if (0u != (text_dirty_pages & OLED_APP_TEXT_DIRTY_YAW))
        {
            oled_refresh_pages(OLED_APP_PAGE_YAW, OLED_APP_PAGE_YAW);
        }
        if (0u != (text_dirty_pages & OLED_APP_TEXT_DIRTY_RPM))
        {
            oled_refresh_pages(OLED_APP_PAGE_RPM, OLED_APP_PAGE_RPM);
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
