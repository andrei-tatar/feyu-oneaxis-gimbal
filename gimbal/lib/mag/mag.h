#ifndef _MAG_H_
#define _MAG_H_

#include "spi.h"

class MagAlpha
{
private:
    SpiDriver &spi;

public:
    MagAlpha(SpiDriver &spi);

    double readAngle();
};

#endif