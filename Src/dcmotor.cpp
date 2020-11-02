#include "dcmotor.h"
#include "string.h" // for memset()
#include "utility.h"


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
                 GPIO_TypeDef* _reversePort,  uint16_t _reversePin,
                 GPIO_TypeDef* _pwmPort,  uint16_t _pwmPin, TIM_TypeDef* _pwmTimer)
    : pwmTimer(_pwmTimer)
    , forwardPort(_forwardPort)
    , reversePort(_reversePort)
    , pwmPort(_pwmPort)
    , pwmPin(_pwmPin)
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

    if(pwmTimer == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // TIM3 GPIO Configuration
        // PA6     ------> TIM3_CH1
        // PA7     ------> TIM3_CH2
        GPIO_InitStruct.Pin       = pwmPin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(pwmPort, &GPIO_InitStruct);

        TIM_ClockConfigTypeDef sClockSourceConfig;
        TIM_MasterConfigTypeDef sMasterConfig;
        memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
        memset(&sMasterConfig, 0, sizeof(sMasterConfig));

        htimPWM.Instance = pwmTimer;
        htimPWM.Init.Prescaler         = 240;
        htimPWM.Init.CounterMode       = TIM_COUNTERMODE_UP;
        htimPWM.Init.Period            = 1000;
        htimPWM.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV4;
        htimPWM.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if(HAL_TIM_Base_Init(&htimPWM) != HAL_OK) {
            Error_Handler();
        }

        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if(HAL_TIM_ConfigClockSource(&htimPWM, &sClockSourceConfig) != HAL_OK) {
            Error_Handler();
        }

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
        if(HAL_TIMEx_MasterConfigSynchronization(&htimPWM, &sMasterConfig) != HAL_OK) {
            Error_Handler();
        }

        HAL_TIM_PWM_Start(&htimPWM, TIM_CHANNEL_ALL);
    }
    else {
        Error_Handler();
    }

}


void
DcMotor::stop() {
    HAL_GPIO_WritePin(forwardPort, forwardPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(reversePort, reversePin, GPIO_PIN_RESET);
    if(pwmPin == GPIO_PIN_6)
        pwmTimer->CCR1 = 0;
    else
        pwmTimer->CCR2 = 0;
}


void
DcMotor::goForward(double speed) {
    HAL_GPIO_WritePin(forwardPort, forwardPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(reversePort, reversePin, GPIO_PIN_RESET);
    if(pwmPin == GPIO_PIN_6)
        pwmTimer->CCR1 = speed;
    else
        pwmTimer->CCR2 = speed;
}


void
DcMotor::goBackward(double speed) {
    HAL_GPIO_WritePin(forwardPort, forwardPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(reversePort, reversePin, GPIO_PIN_SET);
    if(pwmPin == GPIO_PIN_6)
        pwmTimer->CCR1 = speed;
    else
        pwmTimer->CCR2 = speed;
}


