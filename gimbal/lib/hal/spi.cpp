#include "spi.h"

SpiDriver Spi_Hall(SPI1, Hall_CS), Spi_Imu(SPI3, Imu_CS);

SpiDriver::SpiDriver(SPI_TypeDef *instance, GpioDriver &cs)
    : cs(cs)
{
    spi.Instance = instance;
}

void SpiDriver::initGpioForSpi(GPIO_TypeDef *gpio, uint8_t pin, uint32_t af)
{
    GPIO_InitTypeDef init;
    init.Pin = (1 << pin);
    init.Mode = GPIO_MODE_AF_PP;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    init.Pull = GPIO_NOPULL;
    init.Alternate = af;
    HAL_GPIO_Init(gpio, &init);
}

void SpiDriver::begin()
{
    cs.begin();
    cs.set();

    if (spi.Instance == SPI1)
    {
        initGpioForSpi(GPIOA, 5, GPIO_AF5_SPI1);
        initGpioForSpi(GPIOA, 6, GPIO_AF5_SPI1);
        initGpioForSpi(GPIOA, 7, GPIO_AF5_SPI1);
        __HAL_RCC_SPI1_CLK_ENABLE();
        spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    }
    else if (spi.Instance == SPI3)
    {
        initGpioForSpi(GPIOB, 3, GPIO_AF6_SPI3);
        initGpioForSpi(GPIOB, 4, GPIO_AF6_SPI3);
        initGpioForSpi(GPIOB, 5, GPIO_AF6_SPI3);
        __HAL_RCC_SPI3_CLK_ENABLE();
        spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    }

    spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spi.Init.CLKPhase = SPI_PHASE_2EDGE;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.Mode = SPI_MODE_MASTER;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.CRCPolynomial = 0;
    HAL_SPI_Init(&spi);
}

void SpiDriver::transfer(uint8_t *data, uint16_t size)
{
    HAL_SPI_TransmitReceive(&spi, data, data, size, 1000);
}
