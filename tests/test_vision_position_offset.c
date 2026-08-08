#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control_config.h"
#include "vision_link.h"

uint32 heartbeat_get_ms(void)
{
    return 0u;
}

uint8 uart3_maix_hw_read_byte(uint8 *data)
{
    (void)data;
    return 0u;
}

uint32 uart3_maix_hw_get_rx_count(void)
{
    return 0u;
}

uint32 uart3_maix_hw_get_rx_overflow_count(void)
{
    return 0u;
}

int main(void)
{
    assert(fabsf(vision_link_get_position_offset_m() -
                 BALANCE_VISION_POSITION_OFFSET_M) < 0.0001f);

    vision_link_set_position_offset_m(0.002f);
    assert(fabsf(vision_link_correct_position_m(80) - 0.010f) < 0.0001f);

    vision_link_set_position_offset_m(1.0f);
    assert(vision_link_get_position_offset_m() ==
           VISION_LINK_POSITION_OFFSET_LIMIT_M);
    vision_link_set_position_offset_m(-1.0f);
    assert(vision_link_get_position_offset_m() ==
           -VISION_LINK_POSITION_OFFSET_LIMIT_M);

    puts("vision position offset tests passed");
    return 0;
}
