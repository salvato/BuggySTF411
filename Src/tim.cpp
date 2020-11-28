/*
//=========================================
// On Board Peripherals
//=========================================
// PA5  (CN10 11)   ------> Led On Board
// PC13 (CN7  23)   ------> Button On Board


//===================================
// USART2 GPIO Configuration
//===================================
// GND (CN10 20)    ------> GND
// PA2 (CN10 35)    ------> USART2_TX
// PA3 (CN10 37)    ------> USART2_RX


//==================================
// I2C1 GPIO Configuration
//==================================
// VIN (CN7  16)    ------> +3.3V
// GND (CN7  20)    ------> GND
// PB8 (CN10  3)    ------> I2C1_SCL
// PB9 (CN10  5)    ------> I2C1_SDA


//================================================
// TIM1 (32 Bit) GPIO Configuration (Left Encoder)
//================================================
// PA8 (CN10 21)    ------> TIM1_CH1
// PA9 (CN10 23)    ------> TIM1_CH2


//=================================================
// TIM4 (32 Bit) GPIO Configuration (Right Encoder)
//=================================================
// PB6 (CN10 17)    ------> TIM4_CH1
// PB7 (CN7  21)    ------> TIM4_CH2


//=============================================
// TIM2 GPIO Configuration (Periodic Interrupt)
//=============================================
// Channel2         ------> Motors Sampling
// Channel3         ------> Sonar Sampling
// Channel4         ------> AHRS Sampling


//==================================
// TIM3 GPIO Configuration (PWM)
//==================================
// PA6 (CN10 13)    ------> TIM3_CH1
// PA7 (CN10 15)    ------> TIM3_CH2


//==========================================
// TIM5 GPIO Configuration (Sonar)
//==========================================
// PA0 (CN7 - 28)  ------> TIM3_CH1 (Pulse)
// PA1 (CN7 - 30)  ------> TIM3_CH2 (Echo)


//====================================
// Left Motor Direction Pins
//====================================
// PC8  (CN10  2)    ------> LM298 IN1
// PC9  (CN10  1)    ------> LM298 IN2
//====================================


//====================================
// Right Motor Direction Pins
//====================================
// PC10 (CN7  1)    ------> LM298 IN3
// PC11 (CN7  2)    ------> LM298 IN4
//====================================

//====================================
// Used Peripherals:
// Timers:
//     TIM1 ---> Encoder (Left)
//     TIM2 ---> Periodic Interrupt
//     TIM3 ---> PWM (Motors)
//     TIM4 ---> Encoder (Right)
//     TIM5 ---> Ultrasound Sensor
//====================================
*/


#include "tim.h"
#include "utility.h"
#include "string.h" // for memset()
#include "stm32f4xx_ll_tim.h"


// Defined in main.cpp
extern TIM_HandleTypeDef hLeftEncodertimer;
extern TIM_HandleTypeDef hSamplingTimer;
extern TIM_HandleTypeDef hPwmTimer;
extern TIM_HandleTypeDef hRightEncodertimer;
extern TIM_HandleTypeDef hSonarTimer; // To Measure the Radar Echo Pulse Duration


// Defined in main.cpp
extern double periodicCounterClock;     // 10MHz
extern double sonarTimerClockFrequency; // 10MHz (100ns period)
extern double sonarPulseDelay;          // in seconds
extern double sonarPulseWidth;          // in seconds



// Left Encoder (TIM1 - Advanced Timer)
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
    hLeftEncodertimer.Init.Period            = 0xFFFF;
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
    if(HAL_TIM_Encoder_Init(&hLeftEncodertimer, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&hLeftEncodertimer, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}


// (3 different) Periodic Interrupts (TIM2 - 32 bit General Purpose Timer No GPIO needed)
void
SamplingTimerInit(uint32_t AHRSSamplingPeriod,
                  uint32_t motorSamplingPeriod,
                  uint32_t sonarSamplingPeriod)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Prescaler value to have a 10MHz TIM2 Input Counter Clock
    uint32_t uwPrescalerValue = (uint32_t) (SystemCoreClock/periodicCounterClock)-1;

    hSamplingTimer.Instance = TIM2;
    hSamplingTimer.Init.Period            = 0xFFFFFFFF;             // ARR register (32 bit)
    hSamplingTimer.Init.Prescaler         = uwPrescalerValue;       // PSC
    hSamplingTimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; // (0) No Clock Division
    hSamplingTimer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    hSamplingTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_OC_Init(&hSamplingTimer) != HAL_OK) {
        Error_Handler();
    }

    TIM_ClockConfigTypeDef sClockSourceConfig;
    memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if(HAL_TIM_ConfigClockSource(&hSamplingTimer, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    TIM_OC_InitTypeDef sConfigOC;
    memset(&sConfigOC, 0, sizeof(sConfigOC));
    sConfigOC.OCMode     = TIM_OCMODE_TIMING;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    sConfigOC.Pulse = motorSamplingPeriod; // CCR register
    if(HAL_TIM_OC_ConfigChannel(&hSamplingTimer, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.Pulse = sonarSamplingPeriod;
    if(HAL_TIM_OC_ConfigChannel(&hSamplingTimer, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.Pulse = AHRSSamplingPeriod;
    if(HAL_TIM_OC_ConfigChannel(&hSamplingTimer, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
}


// Motors PWMs (TIM3 - 16 bit General Purpose Timer)
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
    if(HAL_TIM_PWM_Init(&hPwmTimer) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse      = 128;

    if (HAL_TIM_PWM_ConfigChannel(&hPwmTimer, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&hPwmTimer, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&hPwmTimer, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&hPwmTimer, TIM_CHANNEL_2) != HAL_OK) {
      Error_Handler();
    }
}


// Right Encoder (TIM4 - 16 bit General Purpose Timer)
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
    hRightEncodertimer.Init.Period            = 0xFFFF;
    hRightEncodertimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    hLeftEncodertimer.Init.RepetitionCounter  = 0;
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
    if(HAL_TIM_Encoder_Init(&hRightEncodertimer, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&hRightEncodertimer, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}


// Sonar radar
void
SonarTimerInit(void) {
    initTim5GPIO();

    uint32_t uwPrescalerValue = (uint32_t) (SystemCoreClock/sonarTimerClockFrequency)-1;
    uint16_t PulseWidthNumber = sonarPulseWidth*sonarTimerClockFrequency;
    uint16_t PulseDelayNumber = sonarPulseDelay*sonarTimerClockFrequency;
    uint16_t period = PulseWidthNumber+PulseDelayNumber;

    __HAL_RCC_TIM5_CLK_ENABLE();

    memset(&hSonarTimer, 0, sizeof(hSonarTimer));
    hSonarTimer.Instance = TIM5;
    hSonarTimer.Init.Prescaler         = uwPrescalerValue;
    hSonarTimer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    hSonarTimer.Init.Period            = period; // Pulse Total
    hSonarTimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    hSonarTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if(HAL_TIM_OnePulse_Init(&hSonarTimer, TIM_OPMODE_SINGLE) != HAL_OK) {
        Error_Handler();
    }

    LL_TIM_OC_SetCompareCH1(TIM5, PulseDelayNumber); // Pulse Delay
    LL_TIM_OC_SetMode(TIM5,  LL_TIM_CHANNEL_CH1,  LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_ConfigOutput(TIM5, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH | LL_TIM_OCIDLESTATE_LOW);

    // Enable the capture/compare interrupt for channel 1
    LL_TIM_EnableIT_CC1(TIM5);

    //*************************+
    // Start pulse generation  |
    //*************************+
    LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableAllOutputs(TIM5);
    LL_TIM_GenerateEvent_UPDATE(TIM5); // Force update generation

    //*******************************************+
    // Program Channel 2 to act as Input Compare |
    //*******************************************+
    if(HAL_TIM_IC_Init(&hSonarTimer) != HAL_OK) {
      Error_Handler();
    }

    TIM_IC_InitTypeDef sConfigIC;
    memset(&sConfigIC, 0, sizeof(sConfigIC));

    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if(HAL_TIM_IC_ConfigChannel(&hSonarTimer, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    //*****************************************************+
    // Configure the NVIC to handle TIM5 Global Interrupt  |
    //*****************************************************+
    if(HAL_TIM_IC_Start_IT(&hSonarTimer, TIM_CHANNEL_2) != HAL_OK) {
      Error_Handler();
    }

    NVIC_SetPriority(TIM5_IRQn, 0);
    NVIC_EnableIRQ(TIM5_IRQn); // The Global Interrupt
}


void
initTim1GPIO() { // Left Encoder
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // TIM1 GPIO Configuration:
    //    PA8     ------> TIM1_CH1 (CN10 - 23)
    //    PA9     ------> TIM1_CH2 (CN10 - 21)
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void
initTim3GPIO() { // Motors PWMs
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
initTim4GPIO() { // Right Encoder
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // TIM4 GPIO Configuration
    //    PB6     ------> TIM4_CH1 (CN10 - 17)
    //    PB7     ------> TIM4_CH2 (CN7  - 21)
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


void
initTim5GPIO() { // Sonar radar
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // TIM5 GPIO Configuration
    //    PA0  ------> TIM5_CH1 (Pulse Output            CN7  - 28)
    //    PA1  ------> TIM5_CH2 (Echo Input 5V Tolerant  CN7  - 30)
    GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
