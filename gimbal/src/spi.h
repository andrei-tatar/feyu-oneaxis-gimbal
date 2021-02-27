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
    void transfer(uint16_t *data, uint16_t size);
    uint16_t transferSingle(uint16_t data);
};

extern SpiDriver Spi_Hall, Spi_Imu;

#endif
