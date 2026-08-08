#ifndef BUZZER_H_
#define BUZZER_H_

#include "zf_common_typedef.h"

#define BUZZER_COMPLETION_BEEP_COUNT    (3u)
#define BUZZER_COMPLETION_ON_MS         (120u)
#define BUZZER_COMPLETION_GAP_MS        (100u)

void buzzer_init(void);
void buzzer_process(void);
void buzzer_play_completion(void);
void buzzer_stop(void);
uint8 buzzer_is_playing(void);

#endif
