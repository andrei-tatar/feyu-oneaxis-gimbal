#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "gpio.h"
#include "stm32f3xx_hal_tim_ex.h"

class Motor
{
private:
    TIM_HandleTypeDef timer;

public:
    void begin();

    void update();
};

#endif