#include "mag.h"

MagAlpha::MagAlpha(SpiDriver &spi) : spi(spi)
{
}

double MagAlpha::readAngle()
{
    uint8_t data[2] = {0, 0};
    spi.select();
    spi.transfer(data, sizeof(data));
    spi.release();
    double angle = data[0] << 8 | data[1];
    angle = angle * 360 / 65535;
    return angle;
}

void MagAlpha::begin()
{
    spi.begin();
}
