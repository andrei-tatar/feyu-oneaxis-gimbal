#include "hal.h"
#include "mag.h"

int main(void)
{
    init();

    auto mag = MagAlpha(Spi_Hall);
    while (1)
    {
        Serial.printf("angle: %f\n", mag.readAngle());
        HAL_Delay(300);
    }
}
