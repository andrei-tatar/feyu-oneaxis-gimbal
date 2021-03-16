#include "motor.h"
#include "stm32f3xx_hal_tim_ex.h"
#include <math.h>

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

void Motor::update()
{
    uint32_t now = HAL_GetTick();

    static uint32_t next = 0;
    static uint32_t step = 0;
    if (now >= next)
    {
        next = now + 1;

        step++;
        if (step == 768)
        {
            step = 0;
        }

        uint32_t phase = step / 256;
        float angle = ((step % 256) / 256.0) * M_PI / 2;

        float power = .2;

        float a = cosf(angle) * 255.0 * power + 127 * (1 - power);
        float b = sinf(angle) * 255.0 * power + 127 * (1 - power);
        float c = 1 * power + 127 * (1 - power);

        if (phase == 0)
        {
            __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_1, a);
            __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_2, b);
            __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_3, c);
        }
        else if (phase == 1)
        {
            __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_1, c);
            __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_2, a);
            __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_3, b);
        }
        else if (phase == 2)
        {
            __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_1, b);
            __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_2, c);
            __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_3, a);
        }
    }
}