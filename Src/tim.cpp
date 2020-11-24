#include "tim.h"
#include "utility.h"
#include "string.h" // for memset()


TIM_HandleTypeDef hLeftEncodertimer;
TIM_HandleTypeDef hSamplingTimer;
TIM_HandleTypeDef hPwmTimer;
TIM_HandleTypeDef hRightEncodertimer;
TIM_HandleTypeDef hSonarTimer; // To Measure the Radar Echo Pulse Duration

double periodicCounterClock = 1.0e6;// 1MHz


// Left Encoder
void
LeftEncoderTimerInit() {
    __HAL_RCC_TIM1_CLK_ENABLE();
    initTim1GPIO();

    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    memset(&sConfig, 0, sizeof(sConfig));
    memset(&sMasterConfig, 0, sizeof(sMasterConfig));

    hLeftEncodertimer.Instance = TIM1;
    hLeftEncodertimer.Init.Prescaler         = 0;
    hLeftEncodertimer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    hLeftEncodertimer.Init.Period            = 65535;
    hLeftEncodertimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    hLeftEncodertimer.Init.RepetitionCounter = 0;
    hLeftEncodertimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    //sConfig.EncoderMode  = TIM_ENCODERMODE_TI1;
    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 0;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 0;
    HAL_TIM_Encoder_Init(&hLeftEncodertimer, &sConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&hLeftEncodertimer, &sMasterConfig);
}


// Right Encoder
void
RightEncoderTimerInit(void) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    initTim4GPIO();

    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    memset(&sConfig, 0, sizeof(sConfig));
    memset(&sMasterConfig, 0, sizeof(sMasterConfig));

    hRightEncodertimer.Instance = TIM4;
    hRightEncodertimer.Init.Prescaler         = 0;
    hRightEncodertimer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    hRightEncodertimer.Init.Period            = 65535;
    hRightEncodertimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    hRightEncodertimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    //sConfig.EncoderMode  = TIM_ENCODERMODE_TI1;
    sConfig.EncoderMode  = TIM_ENCODERMODE_TI2;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 0;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 0;
    HAL_TIM_Encoder_Init(&hRightEncodertimer, &sConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&hRightEncodertimer, &sMasterConfig);
}


// Periodic Interrupts (3 different - No GPIO needed)
void
SamplingTimerInit(uint32_t AHRSSamplingPeriod,
                  uint32_t motorSamplingPeriod,
                  uint32_t sonarSamplingPeriod)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Prescaler value to have a 1MHz TIM2 Input Counter Clock
    uint32_t uwPrescalerValue = (uint32_t) (SystemCoreClock/periodicCounterClock)-1;

    hSamplingTimer.Instance = TIM2;
    hSamplingTimer.Init.Period            = 0xFFFFFFFF;             // ARR register
    hSamplingTimer.Init.Prescaler         = uwPrescalerValue;       // PSC
    hSamplingTimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; // (0) No Clock Division
    hSamplingTimer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    hSamplingTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_OC_Init(&hSamplingTimer);

    TIM_ClockConfigTypeDef sClockSourceConfig;
    memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&hSamplingTimer, &sClockSourceConfig);

    TIM_OC_InitTypeDef sConfigOC;
    memset(&sConfigOC, 0, sizeof(sConfigOC));
    sConfigOC.OCMode     = TIM_OCMODE_TIMING;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    sConfigOC.Pulse = AHRSSamplingPeriod;
    HAL_TIM_OC_ConfigChannel(&hSamplingTimer, &sConfigOC, TIM_CHANNEL_1);

    sConfigOC.Pulse = motorSamplingPeriod;
    HAL_TIM_OC_ConfigChannel(&hSamplingTimer, &sConfigOC, TIM_CHANNEL_2);

    sConfigOC.Pulse = sonarSamplingPeriod;
    HAL_TIM_OC_ConfigChannel(&hSamplingTimer, &sConfigOC, TIM_CHANNEL_3);
}


// Motors PWMs
void
PwmTimerInit(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    initTim3GPIO();

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig ;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
    memset(&sSlaveConfig,       0, sizeof(sSlaveConfig));
    memset(&sMasterConfig,      0, sizeof(sMasterConfig));
    memset(&sConfigOC,          0, sizeof(sConfigOC));

    // Compute the prescaler value to have TIM3 counter clock equal to 128KHz
    uint32_t uhPrescalerValue = (uint32_t)(SystemCoreClock / 128000) - 1;

    hPwmTimer.Instance = TIM3;
    hPwmTimer.Init.Prescaler         = uhPrescalerValue;
    hPwmTimer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    hPwmTimer.Init.Period            = 254;
    hPwmTimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    hPwmTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Base_Init(&hPwmTimer) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if(HAL_TIM_ConfigClockSource(&hPwmTimer, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if(HAL_TIM_PWM_Init(&hPwmTimer) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&hPwmTimer, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse      = 0;

    if (HAL_TIM_PWM_ConfigChannel(&hPwmTimer, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&hPwmTimer, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}


void
initTim1GPIO() {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
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


void
initTim3GPIO() {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // TIM3 GPIO Configuration
    //    PA6 ------> TIM3_CH1 (Output CN10 - 13)
    //    PA7 ------> TIM3_CH2 (Input  CN10 - 15)
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void
initTim4GPIO() {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
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
