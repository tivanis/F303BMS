#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f3xx_hal.h"

extern TIM_HandleTypeDef htim2;

//IMPORTANT: initialize timer htim2 to tick with 1 microsecond period with prescaler
//else this won't work properly
static inline void delay_us(uint32_t delayInMicroseconds)
{
	TIM_HandleTypeDef *handle=&htim2;
	// Use 1 second as maximum delay for now
	if(delayInMicroseconds <= 999999)
	{
		handle->Instance->CNT=0;
		HAL_TIM_Base_Start(handle);
		//TODO: trim for delay start overhead
		while(__HAL_TIM_GET_COUNTER(handle) < delayInMicroseconds)
		{

		}
		HAL_TIM_Base_Stop(handle);
		//Return the previous function state
	}
}

#endif
