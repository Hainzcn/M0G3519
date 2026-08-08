#include "buzzer.h"

#include "buzzer_hw.h"
#include "heartbeat.h"

typedef enum
{
    BUZZER_STATE_IDLE = 0,
    BUZZER_STATE_ON,
    BUZZER_STATE_GAP,
} buzzer_state_enum;

static buzzer_state_enum buzzer_state;
static uint8 buzzer_beeps_remaining;
static uint32 buzzer_phase_start_ms;

void buzzer_init(void)
{
    buzzer_hw_init();
    buzzer_state = BUZZER_STATE_IDLE;
    buzzer_beeps_remaining = 0u;
    buzzer_phase_start_ms = heartbeat_get_ms();
}

void buzzer_process(void)
{
    uint32 now_ms;

    if (BUZZER_STATE_IDLE == buzzer_state)
    {
        return;
    }

    now_ms = heartbeat_get_ms();
    if (BUZZER_STATE_ON == buzzer_state)
    {
        if ((now_ms - buzzer_phase_start_ms) < BUZZER_COMPLETION_ON_MS)
        {
            return;
        }

        buzzer_hw_set_enabled(0u);
        buzzer_beeps_remaining--;
        buzzer_phase_start_ms = now_ms;
        buzzer_state = (0u == buzzer_beeps_remaining) ?
            BUZZER_STATE_IDLE : BUZZER_STATE_GAP;
        return;
    }

    if ((now_ms - buzzer_phase_start_ms) >= BUZZER_COMPLETION_GAP_MS)
    {
        buzzer_hw_set_enabled(1u);
        buzzer_phase_start_ms = now_ms;
        buzzer_state = BUZZER_STATE_ON;
    }
}

void buzzer_play_completion(void)
{
    buzzer_beeps_remaining = BUZZER_COMPLETION_BEEP_COUNT;
    buzzer_phase_start_ms = heartbeat_get_ms();
    buzzer_state = BUZZER_STATE_ON;
    buzzer_hw_set_enabled(1u);
}

void buzzer_stop(void)
{
    buzzer_hw_set_enabled(0u);
    buzzer_state = BUZZER_STATE_IDLE;
    buzzer_beeps_remaining = 0u;
}

uint8 buzzer_is_playing(void)
{
    return (BUZZER_STATE_IDLE != buzzer_state) ? 1u : 0u;
}
