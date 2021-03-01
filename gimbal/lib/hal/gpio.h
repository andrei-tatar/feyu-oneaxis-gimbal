#ifndef _GPIO_H_
#define _GPIO_H_

#include "stm32f3xx_hal.h"

class GpioDriver
{
private:
    GPIO_TypeDef *gpio;
    uint32_t pin, mode;

public:
    GpioDriver(GPIO_TypeDef *gpio, uint8_t pin, uint32_t mode = GPIO_MODE_OUTPUT_PP);
    void begin();

    inline void set()
    {
        HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
    }

    inline void reset()
    {
        HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);
    }

    inline void toggle()
    {
        HAL_GPIO_TogglePin(gpio, pin);
    }

    inline bool read()
    {
        return HAL_GPIO_ReadPin(gpio, pin);
    }

    inline void write(uint8_t st)
    {
        HAL_GPIO_WritePin(gpio, pin, (GPIO_PinState)st);
    }
};

extern GpioDriver Hall_CS, Imu_CS;

#endif