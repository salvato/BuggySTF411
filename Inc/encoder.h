#pragma once

#include <stm32f4xx_hal.h>

class Encoder
{
public:
    Encoder(TIM_TypeDef* _timer);
    void    init();
    void    start();
    double  read();
    void    reset();
    double  readAndReset();
    int32_t readTotal();
    void    resetTotal();
    int32_t readAndResetTotal();

private:
    void    initGPIO();

private:
    TIM_TypeDef*      timer;
    TIM_HandleTypeDef htimer;
    uint32_t          total;
};
