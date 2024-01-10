#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "stm32f7xx.h"

extern TIM_HandleTypeDef htim5;


Model::Model() : modelListener(0)
{

}

void Model::tick()
{

}

void Model::pwm(int value){
	TIM5->CCR1=value;
}

