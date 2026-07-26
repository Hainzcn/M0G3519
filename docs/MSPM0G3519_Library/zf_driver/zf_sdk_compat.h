#ifndef ZF_SDK_COMPAT_H_
#define ZF_SDK_COMPAT_H_

#include <ti/devices/msp/msp.h>

/* SeekFree-added DriverLib helper (not in TI SDK 2.10).
 * Avoid including dl_timer.h here — it emits a TI #warning when used directly. */
void DL_Timer_Count_CCP(GPTIMER_Regs *gptimer);

#endif /* ZF_SDK_COMPAT_H_ */
