#include "serial.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

SerialDriver Serial;

void SerialDriver::begin(uint32_t baud)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef tx;
    tx.Pin = GPIO_PIN_6;
    tx.Mode = GPIO_MODE_AF_PP;
    tx.Alternate = GPIO_AF7_USART1;
    tx.Speed = GPIO_SPEED_FREQ_LOW;
    tx.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &tx);

    // GPIO_InitTypeDef rx;
    // rx.Pin = GPIO_PIN_7;
    // rx.Mode = GPIO_MODE_AF_OD;
    // rx.Alternate = GPIO_AF7_USART1;
    // rx.Speed = GPIO_SPEED_FREQ_LOW;
    // rx.Pull = GPIO_PULLUP;
    // HAL_GPIO_Init(GPIOB, &rx);

    handle.Instance = USART1;
    handle.Init.BaudRate = baud;
    handle.Init.WordLength = USART_WORDLENGTH_8B;
    handle.Init.StopBits = USART_STOPBITS_1;
    handle.Init.Parity = USART_PARITY_NONE;
    handle.Init.Mode = USART_MODE_TX;
    HAL_USART_Init(&handle);

    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void SerialDriver::print(const char *msg)
{
    while (handle.State != HAL_USART_STATE_READY)
    {
        //wait previous tx to finish
    }

    uint16_t size = strlen(msg);
    if (size > sizeof(txBuffer))
        size = sizeof(txBuffer);
    memcpy(txBuffer, msg, size);
    HAL_USART_Transmit_IT(&handle, txBuffer, size);
}

void SerialDriver::printf(const char *fmt, ...)
{
    char buffer[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    print(buffer);
}

extern "C" void USART1_IRQHandler(void)
{
    HAL_USART_IRQHandler(&Serial.handle);
}
