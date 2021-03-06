#include "hal.h"
#include "mag.h"
#include "mpu.h"

int main(void)
{
    init();
    HAL_Delay(1000);

    auto mpu = MPU6500FIFO(Spi_Imu);
    auto mag = MagAlpha(Spi_Hall);

    auto result = mpu.begin();
    Serial.printf("MPU begin: %d\n", result);
    result = mpu.setSrd(0);
    Serial.printf("SRD set: %d\n", result);
    result = mpu.enableFifo(true, true, false);
    Serial.printf("FIFO enable: %d\n", result);
    result = mpu.enableDataReadyInterrupt();
    Serial.printf("DRDY enable: %d\n", result);

    uint16_t readsPerSec = 0;
    uint32_t next = 0;

    while (1)
    {
        if (mpu.available() && mpu.readFifo() == 1)
        {
            readsPerSec++;
        }

        uint32_t now = HAL_GetTick();
        if (now >= next)
        {
            next = now + 1000;
            Serial.printf("cps: %d; a: %f %f %f; g: %f %f %f\n", readsPerSec);
            readsPerSec = 0;
        }
    }
}
