#ifndef _USART_H_
#define _USART_H_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_usart.h"

class SerialDriver
{
private:
    uint8_t txBuffer[256];

public:
    void begin(uint32_t baud = 115200);
    void print(const char *msg);
    void printf(const char *fmt, ...);
    USART_HandleTypeDef handle;
};

extern SerialDriver Serial;

#endif
