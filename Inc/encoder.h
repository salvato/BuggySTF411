#pragma once

#include <stm32f4xx_hal.h>

class Encoder
{
public:
    Encoder(TIM_HandleTypeDef* hEncodertimer);
    void    start();
    double  read();
    void    reset();
    double  readAndReset();
    int32_t readTotal();
    void    resetTotal();
    int32_t readAndResetTotal();

private:
    TIM_HandleTypeDef* htimer;
    uint32_t           encoderChannel;
    uint32_t           total;
};
