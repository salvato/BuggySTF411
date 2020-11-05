// To configure QtCreator so to Debug and Run programs on STM32F boards see:
// https://github.com/nlhans/qt-baremetal
// https://electronics.stackexchange.com/questions/212018/debugging-an-arm-stm32-microcontroller-using-qt-creator
//
// To see the console output:
//      Open a terminal and digit: cat /dev/ttyACM0
//
// Motor Gear 9:1

#include "main.h"
#include "utility.h"
#include "string.h" // for memset()
#include "stdio.h"
#include "encoder.h"
#include "dcmotor.h"
#include "PID_v1.h"
#include "controlledmotor.h"
#include <ADXL345.h>
#include <ITG3200.h>
#include <HMC5883L.h>
#include "MadgwickAHRS.h"


#define BAUD_RATE 9600
#define I2C_SPEEDCLOCK 400000 // Hz


//==================
// Private variables
//==================
bool bAHRSpresent;

I2C_HandleTypeDef hi2c1;

Encoder leftEncoder(TIM1);
Encoder rightEncoder(TIM4);

// DcMotor(forwardPort, forwardPin,
//         reversePort,  reversePin,
//         pwmPort,  pwmPin, pwmTimer)

DcMotor leftMotor(GPIOC, GPIO_PIN_8,
                  GPIOC, GPIO_PIN_9,
                  GPIOA, GPIO_PIN_6, TIM3);

DcMotor rightMotor(GPIOC, GPIO_PIN_10,
                   GPIOC, GPIO_PIN_11,
                   GPIOA, GPIO_PIN_7, TIM3);

ControlledMotor* pLeftControlledMotor  = nullptr;
ControlledMotor* pRightControlledMotor = nullptr;


ADXL345  Acc;      // 400KHz I2C Capable. Maximum Output Data Rate is 800 Hz
ITG3200  Gyro;     // 400KHz I2C Capable
HMC5883L Magn;     // 400KHz I2C Capable, left at the default 15Hz data Rate
Madgwick Madgwick; // ~13us per Madgwick.update() with NUCLEO-F411RE

static float values[9];


TIM_HandleTypeDef htim2;          // Samplig Timer
uint32_t samplingFrequency = 100; // Sampling Frequency [Hz]
//uint32_t samplingFrequency = 300; // Hz


UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;
uint8_t sMessage[255];

volatile bool bRun;


//====================================
// Private function prototypes
//====================================
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

static bool Sensors_Init();


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
// VIN (CN7  18)    ------> +5V
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
// No GPIO used


//==================================
// TIM3 GPIO Configuration (PWM)
//==================================
// PA6 (CN10 13)    ------> TIM3_CH1
// PA7 (CN10 15)    ------> TIM3_CH2


//====================================
// Left Motor Direction Pins
// PC8  (CN10  2)    ------> LM298 IN1
// PC9  (CN10  1)    ------> LM298 IN2
//====================================


//====================================
// Right Motor Direction Pins
// PC10 (CN7  1)    ------> LM298 IN3
// PC11 (CN7  2)    ------> LM298 IN4
//====================================


int
main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();

    leftEncoder.init();
    leftEncoder.start();
    leftMotor.init();

    rightEncoder.init();
    rightEncoder.start();
    rightMotor.init();

    // Initialize the Serial Communication Port (/dev/ttyACM0)
    MX_USART2_UART_Init();

    // Initialize the Periodic Samplig Timer
    MX_TIM2_Init();

    // 10DOF Sensor Initialization
    bAHRSpresent = Sensors_Init();

    if(bAHRSpresent) {
        Madgwick.begin(float(samplingFrequency));

        // Get the first Sensor data
        while(!Acc.getInterruptSource(7)) {}
        Acc.get_Gxyz(&values[0]);
        while(!Gyro.isRawDataReadyOn()) {}
        Gyro.readGyro(&values[3]);
        while(!Magn.isDataReady()) {}
        Magn.ReadScaledAxis(&values[6]);

        // Initial estimate of the attitude (assumed a static sensor !)
        for(int i=0; i<10000; i++) { // ~13us per Madgwick.update() with NUCLEO-F411RE
            Madgwick.update(values[3], values[4], values[5], // Gyro in degrees/sec
                            values[0], values[1], values[2], // Acc
                            values[6], values[7], values[8]);// Mag
        }
    }

    sprintf((char *)sMessage, "\n\n\nBuggySTF411 - Program Started\n");
    if(HAL_UART_Transmit(&huart2, sMessage, strlen((char *)sMessage), 100) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }

    pLeftControlledMotor  = new ControlledMotor(&leftMotor,  &leftEncoder,  samplingFrequency);
    pRightControlledMotor = new ControlledMotor(&rightMotor, &rightEncoder, samplingFrequency);

    double targetSpeed = 2.0;
    pLeftControlledMotor->setTargetSpeed(targetSpeed);

    HAL_TIM_Base_Start_IT(&htim2);

    bRun = false;

    // Enable and set Button EXTI Interrupt to the lowest priority
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    bool oldStatus = bRun;

    // Main Loop
    while(true) {
        HAL_Delay(300);
        sprintf((char *)sMessage, "Speed: %d\n", int(pLeftControlledMotor->currentSpeed*100.0));
        if(HAL_UART_Transmit(&huart2, sMessage, strlen((char *)sMessage), 100) != HAL_OK) {
            HAL_TIM_Base_Stop_IT(&htim2);
            Error_Handler();
        }
        if(bRun != oldStatus) {
            oldStatus = bRun;
            if(!bRun)
                pLeftControlledMotor->Stop();
            else {
                targetSpeed = -targetSpeed;
                pLeftControlledMotor->setTargetSpeed(targetSpeed);
            }
        }
    }
}


static void
MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed      = I2C_SPEEDCLOCK;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c1) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }
}


static void
MX_TIM2_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
    memset(&sMasterConfig, 0, sizeof(sMasterConfig));

    __HAL_RCC_TIM2_CLK_ENABLE();

    // Time base configuration (based on 100 MHz CPU frequency)
    const uint32_t counterClock = SystemCoreClock/100;// 1MHz;
    // Prescaler value to have a 1MHz TIM2 Input Counter Clock
    uint32_t uwPrescalerValue = (uint32_t) (SystemCoreClock/counterClock)-1;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler         = uwPrescalerValue;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = (counterClock/samplingFrequency)-1; // (Sampling period) / (Clock Period)
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; // tDTS=tCK_INT
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;


    if(HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if(HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}


static void
MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate     = BAUD_RATE;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart2) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }
}


static void
MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}


static void
MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);

    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}


bool
Sensors_Init() {
    bool bResult;

    // Accelerator Init
    bResult = Acc.init(ADXL345_ADDR_ALT_LOW, &hi2c1);
    if(!bResult) {
        return false;
    }
    Acc.setRangeSetting(2); // +/- 2g. Possible values are: 2g, 4g, 8g, 16g

    // Gyroscope Init
    bResult = Gyro.init(ITG3200_ADDR_AD0_LOW, &hi2c1);
    if(!bResult) {
        return false;
    }
    HAL_Delay(100);
    Gyro.zeroCalibrate(600); // calibrate the ITG3200

    // Magnetometer Init
    bResult = Magn.init(HMC5883L_Address, &hi2c1);
    if(!bResult) {
        return false;
    }
    HAL_Delay(100);
    int16_t error = Magn.SetScale(1300); // Set the scale (in milli Gauss) of the compass.
    if(error != 0) {
        return false;
    }
    HAL_Delay(100);
    error = Magn.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
    if(error != 0) {
        return false;
    }
    return true;
}


//  * @brief This function handles TIM2 global interrupt.
void
TIM2_IRQHandler(void) {
    if(pLeftControlledMotor)
        pLeftControlledMotor->Update();
    HAL_TIM_IRQHandler(&htim2);
}


//  * @brief This function handles EXTI line[15:10] interrupts.
void
EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(B1_Pin);
}


void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == B1_Pin) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        bRun = !bRun;
    }
}


void
SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
    memset(&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }
}


#ifdef  USE_FULL_ASSERT
void
assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif // USE_FULL_ASSERT
