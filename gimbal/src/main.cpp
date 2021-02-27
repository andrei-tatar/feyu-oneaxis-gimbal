#include "hal.h"

int main(void)
{
    init();

    uint32_t reg = 6;
    while (1)
    {
        uint16_t readReg = 0x40 | reg;
        readReg <<= 8;
        Serial.printf("RX: %04X %04X\n", Spi_Hall.transferSingle(readReg), Spi_Hall.transferSingle(0));
        HAL_Delay(300);
    }
}
