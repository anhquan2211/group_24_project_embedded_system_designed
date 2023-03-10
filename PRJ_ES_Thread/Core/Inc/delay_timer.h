#ifndef __DELAY_TIMER_H
#define __DELAY_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void TIM_DelayUs(TIM_HandleTypeDef *htim, uint16_t Delay);
void TIM_DelayMs(TIM_HandleTypeDef *htim, uint16_t Delay);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_TIMER_H */
