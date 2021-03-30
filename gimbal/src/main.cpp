#include "hal.h"
#include "mag.h"
#include "mpu.h"
#include "motor.h"

int main(void)
{
    init();
    HAL_Delay(1000);

    auto mag = MagAlpha(Spi_Hall);
    mag.begin();

    // auto mpu = MPU6500FIFO(Spi_Imu);
    // mpu.begin();
    // mpu.setSrd(0);
    // mpu.enableFifo(true, true, false);
    // mpu.enableDataReadyInterrupt();

    // uint16_t readsPerSec = 0;
    // uint32_t next = 0;

    Motor motor(mag);
    motor.begin();
    // motor.calibrate();

    while (1)
    {
        motor.update();

        // if (mpu.available() && mpu.readFifo() == 1)
        // {
        //     readsPerSec++;
        // }

        // uint32_t now = HAL_GetTick();
        // if (now >= next)
        // {
        //     next = now + 1000;
        //     float test[512];
        //     size_t size;
        //     mpu.getFifoGyroZ_rads(&size, test);
        //     Serial.printf("cps: %d; s:%d; a: %f %f %f; g: %f %f %f\n", readsPerSec, size, test[0]);
        //     readsPerSec = 0;
        // }
    }
}
