#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "gpio.h"
#include "mag.h"
#include "stm32f3xx_hal_tim_ex.h"

class Motor
{
private:
    TIM_HandleTypeDef timer;
    MagAlpha &mag;
    float offset = 23.598993;
    float zeroAngles[7] = {0, 52.207831000000006, 100.98786799999999, 149.20760199999998, 199.053335, 251.222693, 307.555975};
    uint8_t zeroAnglesLength = 7;
    float targetAngle = 0;

    void setStep(uint16_t step, float power);

public:
    Motor(MagAlpha &mag);

    void begin();

    void calibrate();
    void update();
    void setTarget(float angle);
};

#endif