#include "button.h"

#include "button_hw.h"
#include "heartbeat.h"

typedef struct
{
    uint8  raw_level;
    uint8  stable_pressed;
    uint32 last_change_ms;
} button_channel_t;

static button_channel_t button_channels[BUTTON_HW_COUNT];
static button_id_t      button_active = BUTTON_ID_NONE;

static const char *const button_names[] =
{
    "",
    "SW1",
    "SW2",
    "SW3",
    "SW4",
};

static uint8 button_channel_is_pressed(uint8 raw_level)
{
    return (raw_level == BUTTON_HW_ACTIVE_LEVEL) ? 1u : 0u;
}

static void button_update_channel(uint8 index, uint32 now_ms)
{
    uint8 raw_level;
    uint8 raw_pressed;
    button_channel_t *channel;

    channel = &button_channels[index];
    raw_level = button_hw_read_raw(index);
    if (raw_level != channel->raw_level)
    {
        channel->raw_level     = raw_level;
        channel->last_change_ms = now_ms;
    }

    raw_pressed = button_channel_is_pressed(channel->raw_level);
    if ((now_ms - channel->last_change_ms) >= BUTTON_DEBOUNCE_MS)
    {
        channel->stable_pressed = raw_pressed;
    }
}

void button_init(void)
{
    uint8 i;

    button_hw_init();
    for (i = 0u; i < BUTTON_HW_COUNT; i++)
    {
        button_channels[i].raw_level       = (uint8)(BUTTON_HW_ACTIVE_LEVEL ^ 1u);
        button_channels[i].stable_pressed  = 0u;
        button_channels[i].last_change_ms  = 0u;
    }
    button_active = BUTTON_ID_NONE;
}

void button_process(void)
{
    uint8 i;
    uint32 now_ms;

    now_ms = heartbeat_get_ms();
    for (i = 0u; i < BUTTON_HW_COUNT; i++)
    {
        button_update_channel(i, now_ms);
    }

    button_active = BUTTON_ID_NONE;
    for (i = 0u; i < BUTTON_HW_COUNT; i++)
    {
        if (0u != button_channels[i].stable_pressed)
        {
            button_active = (button_id_t)(BUTTON_ID_SW1 + i);
            break;
        }
    }
}

button_id_t button_get_active(void)
{
    return button_active;
}

const char *button_get_name(button_id_t id)
{
    if ((uint8)id >= (sizeof(button_names) / sizeof(button_names[0])))
    {
        return "";
    }

    return button_names[(uint8)id];
}
