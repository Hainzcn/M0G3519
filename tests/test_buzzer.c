#include <assert.h>
#include <stdio.h>

#include "buzzer.h"

static uint32 mock_now_ms;
static uint8 mock_buzzer_enabled;
static uint32 mock_buzzer_init_count;
static uint32 mock_buzzer_change_count;

uint32 heartbeat_get_ms(void)
{
    return mock_now_ms;
}

void buzzer_hw_init(void)
{
    mock_buzzer_init_count++;
    mock_buzzer_enabled = 0u;
}

void buzzer_hw_set_enabled(uint8 enabled)
{
    mock_buzzer_enabled = enabled;
    mock_buzzer_change_count++;
}

static void test_completion_pattern(void)
{
    mock_now_ms = 0u;
    mock_buzzer_enabled = 1u;
    mock_buzzer_init_count = 0u;
    mock_buzzer_change_count = 0u;

    buzzer_init();
    assert(mock_buzzer_init_count == 1u);
    assert(mock_buzzer_enabled == 0u);
    assert(buzzer_is_playing() == 0u);

    buzzer_play_completion();
    assert(mock_buzzer_enabled == 1u);
    assert(buzzer_is_playing() != 0u);

    mock_now_ms = BUZZER_COMPLETION_ON_MS - 1u;
    buzzer_process();
    assert(mock_buzzer_enabled == 1u);

    mock_now_ms = BUZZER_COMPLETION_ON_MS;
    buzzer_process();
    assert(mock_buzzer_enabled == 0u);

    mock_now_ms += BUZZER_COMPLETION_GAP_MS;
    buzzer_process();
    assert(mock_buzzer_enabled == 1u);

    mock_now_ms += BUZZER_COMPLETION_ON_MS;
    buzzer_process();
    assert(mock_buzzer_enabled == 0u);

    mock_now_ms += BUZZER_COMPLETION_GAP_MS;
    buzzer_process();
    assert(mock_buzzer_enabled == 1u);

    mock_now_ms += BUZZER_COMPLETION_ON_MS;
    buzzer_process();
    assert(mock_buzzer_enabled == 0u);
    assert(buzzer_is_playing() == 0u);
    assert(mock_buzzer_change_count == 6u);
}

static void test_stop_cancels_pattern(void)
{
    mock_now_ms = 1000u;
    buzzer_play_completion();
    assert(mock_buzzer_enabled == 1u);

    buzzer_stop();
    assert(mock_buzzer_enabled == 0u);
    assert(buzzer_is_playing() == 0u);

    mock_now_ms += BUZZER_COMPLETION_ON_MS + BUZZER_COMPLETION_GAP_MS;
    buzzer_process();
    assert(mock_buzzer_enabled == 0u);
}

int main(void)
{
    test_completion_pattern();
    test_stop_cancels_pattern();
    puts("buzzer tests passed");
    return 0;
}
