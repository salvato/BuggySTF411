#include "dcmotor.h"
#include "string.h" // for memset()


// Caratteristiche Motore:

// Tensione applicabile: 3 - 12 Vdc
// Velocità di rotazione 130RPM @ 12V - ???mA
// Coppia max ???gf @ ?V
// Riduzione 1:9
// Diametro Ruota 69mm

// Resistenza Avvolgimento: ??
// Induttanza Avvolgimento: ??

// http://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling

// (J)     moment of inertia of the rotor     ??.? kg.m^2
// (b)     motor viscous friction constant    ?.?  N.m.s
// (Ke)    electromotive force constant       ?.?? V/rad/sec
// (Kt)    motor torque constant              ?.?? N.m/Amp
// (R)     electric resistance                ?.?  Ohm
// (L)     electric inductance                ?.?  H

DcMotor::DcMotor(GPIO_TypeDef* _forwardPort, uint16_t _forwardPin,
                 GPIO_TypeDef* _reversePort,  uint16_t _reversePin)
    : forwardPort(_forwardPort)
    , reversePort(_reversePort)
    , forwardPin(_forwardPin)
    , reversePin(_reversePin)
{
}


void
DcMotor::init() {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

    if(reversePort == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    else if(reversePort == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if(reversePort == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    else if(reversePort == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
    else if(reversePort == GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE();

    if(forwardPort == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    else if(forwardPort == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if(forwardPort == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    else if(forwardPort == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
    else if(forwardPort == GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitStruct.Pin   = forwardPin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(forwardPort, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = reversePin;
    HAL_GPIO_Init(reversePort, &GPIO_InitStruct);
}


void DcMotor::stop() {

}


void
DcMotor::goForward(double speed) {

}


void DcMotor::goBackward(double speed) {

}


