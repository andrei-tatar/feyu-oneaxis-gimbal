#ifndef _USART_H_
#define _USART_H_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_usart.h"

class SerialDriver
{
private:
    USART_HandleTypeDef usart;

public:
    void begin(uint32_t baud = 9600);
    void print(const char *msg);
    void printf(const char *fmt, ...);
};

extern SerialDriver Serial;

#endif
