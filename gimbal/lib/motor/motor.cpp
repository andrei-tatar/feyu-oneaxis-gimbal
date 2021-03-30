#include "motor.h"
#include "stm32f3xx_hal_tim_ex.h"
#include <math.h>
#include "serial.h"

Motor::Motor(MagAlpha &mag) : mag(mag)
{
}

void Motor::begin()
{
    GPIO_InitTypeDef pwm;
    pwm.Mode = GPIO_MODE_AF_PP;
    pwm.Alternate = GPIO_AF6_TIM1;
    pwm.Speed = GPIO_SPEED_FREQ_HIGH;
    pwm.Pull = GPIO_NOPULL;
    pwm.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &pwm);

    pwm.Alternate = GPIO_AF6_TIM1;
    pwm.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &pwm);

    pwm.Alternate = GPIO_AF4_TIM1;
    pwm.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &pwm);

    __HAL_RCC_TIM1_CLK_ENABLE();

    timer.Instance = TIM1;
    timer.Init.Prescaler = 0;
    timer.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    timer.Init.Period = 256;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.RepetitionCounter = 0;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&timer);

    TIM_OC_InitTypeDef channel;
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.OCIdleState = TIM_OCIDLESTATE_SET;
    channel.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    channel.Pulse = 128;
    channel.OCPolarity = TIM_OCPOLARITY_LOW;
    channel.OCNPolarity = TIM_OCNPOLARITY_LOW;
    channel.OCFastMode = TIM_OCFAST_ENABLE;

    TIM_MasterConfigTypeDef masterConfig;
    masterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    masterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&timer, &masterConfig);

    HAL_TIM_PWM_ConfigChannel(&timer, &channel, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_1);

    HAL_TIM_PWM_ConfigChannel(&timer, &channel, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_2);

    HAL_TIM_PWM_ConfigChannel(&timer, &channel, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_3);
}

void Motor::calibrate()
{
    uint16_t step = 0;

    float startAngle = -1;

    while (true)
    {
        setStep(step, .3);
        HAL_Delay(5);

        auto angle = mag.readAngle();

        //TODO: store in some other array and sort
        if (step == 0)
        {
            Serial.printf("deg %f:\n", angle);
            zeroAngles[zeroAnglesLength++] = angle;
            if (startAngle == -1)
            {
                startAngle = angle;
            }
            else
            {
                if (abs(startAngle - angle) < 5)
                {
                    zeroAnglesLength--;
                    Serial.printf("count %d: ", zeroAnglesLength);
                    for (uint8_t i = 0; i < zeroAnglesLength; i++)
                    {
                        Serial.printf("%f ", zeroAngles[i]);
                    }
                    Serial.printf("\n");
                    return;
                }
            }
        }

        step++;
        if (step == 768)
        {
            step = 0;
        }
    }
}

void Motor::update()
{
    uint32_t now = HAL_GetTick();
    static uint32_t next = 0;
    if (now >= next)
    {
        next = now + 1;

        float angle = mag.readAngle() - offset;
        if (angle < 0)
        {
            angle += 360;
        }

        float error = targetAngle - angle;
        if (error > 180)
        {
            error -= 360;
        }
        else if (error < -180)
        {
            error += 360;
        }

        if (error > 10)
        {
            error = 10;
        }
        else if (error < -10)
        {
            error = -10;
        }

        float test = angle + error;
        float highAngle = 360, lowAngle;
        for (uint8_t i = zeroAnglesLength - 1; i >= 0; i--)
        {
            if (test >= zeroAngles[i])
            {
                lowAngle = zeroAngles[i];
                break;
            }
            else
            {
                highAngle = zeroAngles[i];
            }
        }

        uint16_t step = (test - lowAngle) / (highAngle - lowAngle) * 767;
        // Serial.printf("a:%f, t:%f, e:%f, s:%d - l: %f, h: %f\n", angle, targetAngle, error, step, lowAngle, highAngle);
        setStep(step, abs(error / 20));
    }
}

void Motor::setStep(uint16_t step, float power)
{
    uint8_t phase = (step / 256) % 3;
    float angle = ((step % 256) / 255.0) * M_PI / 2;

    uint8_t phases[3] = {
        (uint8_t)(cosf(angle) * 255.0 * power + 127 * (1 - power)),
        (uint8_t)(127 * (1 - power)),
        (uint8_t)(sinf(angle) * 255.0 * power + 127 * (1 - power)),
    };

    __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_1, phases[(0 + phase) % 3]);
    __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_2, phases[(1 + phase) % 3]);
    __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_3, phases[(2 + phase) % 3]);
}

void Motor::setTarget(float angle)
{
    targetAngle = angle;
}