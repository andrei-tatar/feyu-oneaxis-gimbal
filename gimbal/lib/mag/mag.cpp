#include "mag.h"

MagAlpha::MagAlpha(SpiDriver &spi) : spi(spi)
{
}

double MagAlpha::readAngle()
{
    double angle = spi.transferSingle(0);
    angle = angle * 360 / 65535;
    return angle;
}
