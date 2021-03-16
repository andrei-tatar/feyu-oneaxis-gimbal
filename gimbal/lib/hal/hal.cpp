#include "hal.h"

void initPinForSpi1(GPIO_TypeDef *gpio, uint32_t pin)
{
    GPIO_InitTypeDef init;
    init.Pin = (1 << pin);
    init.Mode = GPIO_MODE_AF_PP;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    init.Pull = GPIO_NOPULL;
    init.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(gpio, &init);
}

void init()
{
    HAL_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    Serial.begin();
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}
