#pragma once

#include <stm32f4xx_hal.h>

class Encoder
{
public:
    Encoder(TIM_TypeDef* _timer);
    void init();
    void start();
    double read();
    void reset();
    double readAndReset();

private:
    TIM_TypeDef*      timer;
    TIM_HandleTypeDef htimer;

private:
    void initGPIO();
};
