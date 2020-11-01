#pragma once

#include <stm32f4xx_hal.h>

class Encoder
{
public:
    Encoder(TIM_TypeDef* _timer);
    void init();
    void start();
    int16_t read();
    void reset();
    int16_t readAndReset();

private:
    TIM_TypeDef*      timer;
    TIM_HandleTypeDef htimer;

private:
    void initGPIO();
};
