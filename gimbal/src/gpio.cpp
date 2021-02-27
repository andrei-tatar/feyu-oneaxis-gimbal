#include "gpio.h"

GpioDriver Hall_CS(GPIOA, 1), Imu_CS(GPIOB, 8);

GpioDriver::GpioDriver(GPIO_TypeDef *gpio, uint8_t pin, uint32_t mode)
    : pin(1 << pin), mode(mode)
{
    this->gpio = gpio;
}

void GpioDriver::begin()
{
    GPIO_InitTypeDef init;
    init.Pin = pin;
    init.Mode = mode;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &init);
}