#include "zf_sdk_compat.h"

/* Dir-encoder path only; dual-drive uses QEI (encoder_quad_init). */
void DL_Timer_Count_CCP(GPTIMER_Regs *gptimer)
{
    (void)gptimer;
}
