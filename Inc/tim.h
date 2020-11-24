#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"


void LeftEncoderTimerInit(void);
void SamplingTimerInit(uint32_t AHRSSamplingPeriod,
                       uint32_t motorSamplingPeriod,
                       uint32_t sonarSamplingPeriod);
void PwmTimerInit(void);
void RightEncoderTimerInit(void);
void EchoTimerInit();

void initTim1GPIO();
void initTim3GPIO();
void initTim4GPIO();
void initTim11GPIO();
#ifdef __cplusplus
}
#endif
