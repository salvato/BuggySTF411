#pragma once

#include <stm32f4xx_hal.h>


class DcMotor
{
public:
    explicit DcMotor(GPIO_TypeDef* _forwardPort, uint16_t _forwardPin,
                     GPIO_TypeDef* _reversePort,  uint16_t _reversePin);
    void init();
    void stop();
    void goForward(double  speed=0.0);
    void goBackward(double speed=0.0);

private:
    GPIO_TypeDef* forwardPort;
    GPIO_TypeDef* reversePort;
    uint16_t forwardPin;
    uint16_t reversePin;

    uint16_t pwmFrequency;
    int forwardPWMrange;
    int backwardPWMrange;
};
