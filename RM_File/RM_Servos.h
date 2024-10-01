#pragma once
#include "RM_stm32fxxx_hal.h"
//舵机
typedef struct
{
	TIM_HandleTypeDef* Tim;
	uint32_t Channel;
}Servos_t;

class RM_Servos
{
public:
  Servos_t Servos;
  void SetAngle(uint32_t angle);
	void Init(TIM_HandleTypeDef* Tim,uint32_t Channel);
};
inline void RM_Servos::Init(TIM_HandleTypeDef* Tim,uint32_t Channel)
{
	Servos.Tim = Tim;
	
	Servos.Channel = Channel;
	
	HAL_TIM_PWM_Start(Servos.Tim,Servos.Channel);
	
	__HAL_TIM_SET_COMPARE(Servos.Tim, Servos.Channel, 500 + 11.1111f * 90);  
}

inline void RM_Servos::SetAngle(uint32_t angle)
{
	__HAL_TIM_SET_COMPARE(this->Servos.Tim, this->Servos.Channel, 500 + 11.1111f * angle);  
}
