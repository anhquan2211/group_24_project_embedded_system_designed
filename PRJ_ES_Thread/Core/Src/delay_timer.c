#include "delay_timer.h"

/**
  * @brief  This function provides minimum delay (in microseconds)
  *    		Blocking mode based on a timer.
  * @param  htim: TIM Base handle of the timer, which is
  * 		the source of time base.
  * @param  Delay: specifies the delay time length, in microseconds.
  * @retval None
  */
void TIM_DelayUs(TIM_HandleTypeDef *htim, uint16_t Delay)
{
	__HAL_TIM_SET_COUNTER(htim, 0);
	HAL_TIM_Base_Start(htim);
	while(__HAL_TIM_GET_COUNTER(htim) < Delay);
	HAL_TIM_Base_Stop(htim);

}

/**
  * @brief  This function provides minimum delay (in milliseconds)
  * 		Blocking mode based on a timer.
  * @param  htim: TIM Base handle of the timer, which is
  * 		the source of time base.
  * @param  Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void TIM_DelayMs(TIM_HandleTypeDef *htim, uint16_t Delay)
{
	while(Delay--)
	{
//		__HAL_TIM_SET_COUNTER(htim, 0);
//		HAL_TIM_Base_Start(htim);
//		while(__HAL_TIM_GET_COUNTER(htim) < 1000);
		TIM_DelayUs(htim, 1000);
	}
//	HAL_TIM_Base_Stop(htim);
}
