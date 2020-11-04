#include "encoder.h"
#include "string.h" // for memset()

// 48 CPR quadrature encoder ??

Encoder::Encoder(TIM_TypeDef *_timer)
    : timer(_timer)
{
}


void
Encoder::start() {
    HAL_TIM_Encoder_Start(&htimer, TIM_CHANNEL_ALL);
}


double
Encoder::read() {
    return double(int16_t(timer->CNT))/48.0;
}


void
Encoder::reset() {
    TIM1->CNT = 0;
}


double Encoder::readAndReset() {
    double value = double(int16_t(timer->CNT))/48.0;
    TIM1->CNT = 0;
    return value;
}


void
Encoder::init() {
    initGPIO();

    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    memset(&sConfig, 0, sizeof(sConfig));
    memset(&sMasterConfig, 0, sizeof(sMasterConfig));

    htimer.Instance = timer;
    htimer.Init.Prescaler         = 0;
    htimer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htimer.Init.Period            = 65535;
    htimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htimer.Init.RepetitionCounter = 0;
    htimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

// TIM_ENCODERMODE_TI1  Quadrature encoder mode 1, x2 mode,
//                      counts up/down on TI1FP1 edge depending
//                      on TI2FP2 level
// TIM_ENCODERMODE_TI2  Quadrature encoder mode 2, x2 mode,
//                      counts up/down on TI2FP2 edge depending
//                      on TI1FP1 level
// TIM_ENCODERMODE_TI12 Quadrature encoder mode 3, x4 mode,
//                      counts up/down on both TI1FP1 and TI2FP2 edges
//                      depending on the level of the other input
    //sConfig.EncoderMode  = TIM_ENCODERMODE_TI1;
    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;

    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 0;

    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter   = 0;
    HAL_TIM_Encoder_Init(&htimer, &sConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htimer, &sMasterConfig);
}


void
Encoder::initGPIO() {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    if(timer == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // TIM1 GPIO Configuration:
        // PA8     ------> TIM1_CH1
        // PA9     ------> TIM1_CH2
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(timer == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        // TIM4 GPIO Configuration
        // PB6     ------> TIM4_CH1
        // PB7     ------> TIM4_CH2
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}
