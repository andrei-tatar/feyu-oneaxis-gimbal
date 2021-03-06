#ifndef _SPI_H_
#define _SPI_H_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_spi.h"
#include "gpio.h"

class SpiDriver
{
private:
    SPI_HandleTypeDef spi;
    GpioDriver &cs;
    void initGpioForSpi(GPIO_TypeDef *gpio, uint8_t pin, uint32_t af);

public:
    SpiDriver(SPI_TypeDef *instance, GpioDriver &cs);
    void begin();
    inline void select() { cs.reset(); }
    inline void release() { cs.set(); }
    void transfer(uint8_t *data, uint16_t size);
};

extern SpiDriver Spi_Hall, Spi_Imu;

#endif
