#include "button_app.h"

#include "button.h"
#include "control_config.h"
#include "oled.h"
#if (BALANCE_CONTROL_ENABLE != 0u)
#include "balance_app.h"
#endif

#define BUTTON_APP_PAGE                 (4)
#define BUTTON_APP_LABEL_X              (0)
#define BUTTON_APP_VALUE_X              (32)

static button_id_t button_app_displayed = BUTTON_ID_NONE;
static button_id_t button_app_previous = BUTTON_ID_NONE;
static uint8       button_app_force_render = 1u;

static void button_app_render(button_id_t active)
{
    oled_clear_page(BUTTON_APP_PAGE);
    oled_show_string(BUTTON_APP_LABEL_X, BUTTON_APP_PAGE, "Key:", OLED_FONT_6X8);
    if (BUTTON_ID_NONE != active)
    {
        oled_show_string(BUTTON_APP_VALUE_X, BUTTON_APP_PAGE,
                         button_get_name(active), OLED_FONT_6X8);
    }
    else
    {
        oled_show_string(BUTTON_APP_VALUE_X, BUTTON_APP_PAGE, "---", OLED_FONT_6X8);
    }
}

void button_app_init(void)
{
    button_init();
    button_app_displayed     = BUTTON_ID_NONE;
    button_app_previous      = BUTTON_ID_NONE;
    button_app_force_render  = 1u;
}

void button_app_process(void)
{
    button_id_t active;

    button_process();

    if (0u == oled_is_ready())
    {
        button_app_force_render = 1u;
        return;
    }

    active = button_get_active();
#if (BALANCE_CONTROL_ENABLE != 0u)
    if ((BUTTON_ID_SW1 == active) && (BUTTON_ID_SW1 != button_app_previous))
    {
        (void)balance_app_start_sequence();
    }
#endif
    button_app_previous = active;
    if ((0u == button_app_force_render) && (active == button_app_displayed))
    {
        return;
    }

    button_app_displayed    = active;
    button_app_force_render = 0u;
    button_app_render(active);
    oled_refresh_pages(BUTTON_APP_PAGE, BUTTON_APP_PAGE);
}
