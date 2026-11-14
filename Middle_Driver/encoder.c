#include "tim.h"
#include "encoder.h"

// 
// @简介：通过定时器计数器来计算电机当前转速并返回，读完后对定时器清零
// @参数：X 倍频（1表示1倍频）
//			 PPR 编码器基础脉冲（11表示一圈11个脉冲）
//			 GR 减速比（103表示电机转103圈负载转一圈）
// 			 
float GetCurrent_Speed(uint8_t X, uint16_t PPR, uint16_t GR)
{
	int16_t cnt = (int16_t)(__HAL_TIM_GET_COUNTER(&htim2));
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	if (X == 0 || PPR == 0 || GR == 0)
	{
		return 0.0f;
	}

	float Current_Speed = (60000.0f * cnt) / (TIME_SIMPLE * X * PPR * GR);
	return Current_Speed;
}
