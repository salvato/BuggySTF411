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


#define BAUD_RATE      9600
#define I2C_SPEEDCLOCK 400000 // Hz


#define USART2_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARt2_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()


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
static volatile float AHRSvalues[9];

TIM_HandleTypeDef htim2;          // Samplig Timer
//uint32_t samplingFrequency = 100; // Sampling Frequency [Hz]
uint32_t samplingFrequency = 300; // Hz

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;
uint8_t txBuffer[255];
uint8_t rxBuffer[255];
volatile bool bTxUartReady;
volatile bool bRxUartReady;

volatile bool bRun;


//====================================
// Private function prototypes
//====================================
void SystemClock_Config(void);

static void GPIO_Init(void);
static void I2C1_Init(void);
static void TIM2_Init(void);
static void USART2_UART_Init(void);

static bool Sensors_Init();
static void AHRS_Init_Position();


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

    GPIO_Init();

// Initialize the Serial Communication Port (/dev/ttyACM0)
    USART2_UART_Init();

    leftEncoder.init();
    leftEncoder.start();
    leftMotor.init();

    rightEncoder.init();
    rightEncoder.start();
    rightMotor.init();

// Initialize the Periodic Samplig Timer
    TIM2_Init();

// 10DOF Sensor Initialization
    I2C1_Init();
    bAHRSpresent = Sensors_Init();

// 10DOF Sensor Position Initialization
    if(bAHRSpresent)
        AHRS_Init_Position();

//    bTxUartReady = false;
//    sprintf((char *)txBuffer, "\n\n\nBuggySTF411 - Program Started\n");
//    if(HAL_UART_Transmit_DMA(&huart2, txBuffer, strlen((char *)txBuffer)) != HAL_OK) {
//        HAL_TIM_Base_Stop_IT(&htim2);
//        Error_Handler();
//    }

    pLeftControlledMotor  = new ControlledMotor(&leftMotor,  &leftEncoder,  samplingFrequency);
    pRightControlledMotor = new ControlledMotor(&rightMotor, &rightEncoder, samplingFrequency);

    double targetSpeed = 2.0;
    HAL_TIM_Base_Start_IT(&htim2);

    // Enable and set Button EXTI Interrupt
    bRun = false;
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    bool oldStatus = bRun;
// DMA is programmed for reception before starting the transmission, in order to
// be sure DMA Rx is ready when counterpart will start transmitting
    float q0, q1, q2, q3;
    bRxUartReady = false;
    if(HAL_UART_Receive_DMA(&huart2, (uint8_t *)rxBuffer, 255) != HAL_OK) {
        Error_Handler();
    }
    bTxUartReady = true;
    //=======================================================================
    //                                Main Loop
    //=======================================================================
    while(true) {
        if(bTxUartReady) {
            Madgwick.getRotation(&q0, &q1, &q2, &q3);
            sprintf((char *)txBuffer, "%4d,%4d,%4d,%4d,%4d,%lu,%ld\n",
                    int(q0*1000.0), int(q1*1000.0), int(q2*1000.0), int(q3*1000.0),
                    int(pLeftControlledMotor->currentSpeed*100.0),
                    long(pLeftControlledMotor->getTotalMove()),
                    ulong(HAL_GetTick()));
            bTxUartReady = false;
            if(HAL_UART_Transmit_DMA(&huart2, txBuffer, strlen((char *)txBuffer)) != HAL_OK) {
                HAL_TIM_Base_Stop_IT(&htim2);
                Error_Handler();
            }
        }
        if(bRun != oldStatus) {
            oldStatus = bRun;
            if(!bRun) {
                pLeftControlledMotor->Stop();
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            }
            else {
                targetSpeed = -targetSpeed;
                pLeftControlledMotor->setTargetSpeed(targetSpeed);
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            }
        }
    } // while(true)
    //=======================================================================
    //                                 End Loop
    //=======================================================================
}


static void
I2C1_Init(void) {
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
TIM2_Init(void) {
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
USART2_UART_Init(void) {
    GPIO_InitTypeDef  GPIO_InitStruct;

    // Enable GPIO TX/RX clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin       = USART_TX_Pin; // GPIO_PIN_2
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = USART_RX_Pin; // GPIO_PIN_3
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable USART2 clock
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance = USART2;
    huart2.Init.BaudRate     = BAUD_RATE;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart2) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }

// Enable DMA1 clock
    __HAL_RCC_DMA1_CLK_ENABLE();

// Configure the DMA streams ##########################################*/
// Configure the DMA handler for Transmission process
    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_usart2_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart2_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_usart2_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
    if(HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }
// Associate the initialized DMA handle to the the UART handle
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

// Configure the DMA handler for Reception process
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode                = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_usart2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_usart2_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart2_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_usart2_rx.Init.PeriphBurst         = DMA_PBURST_INC4;
    if(HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
        HAL_TIM_Base_Stop_IT(&htim2);
        Error_Handler();
    }
    // Associate the initialized DMA handle to the the UART handle
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

// Configure the NVIC for DMA ##########################################*/
// NVIC configuration for DMA Tx transfer complete interrupt
    HAL_NVIC_SetPriority(USART2_DMA_TX_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART2_DMA_TX_IRQn);

// NVIC configuration for DMA Rx transfer complete interrupt
    HAL_NVIC_SetPriority(USART2_DMA_RX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_DMA_RX_IRQn);

// NVIC configuration for USART TC interrupt
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}


static void
GPIO_Init(void) {
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
    // Accelerator Init
    if(!Acc.init(ADXL345_ADDR_ALT_LOW, &hi2c1))
        return false;
    Acc.setRangeSetting(2); // +/- 2g. Possible values are: 2g, 4g, 8g, 16g

    // Gyroscope Init
    if(!Gyro.init(ITG3200_ADDR_AD0_LOW, &hi2c1))
        return false;
    HAL_Delay(100);
    Gyro.zeroCalibrate(600); // calibrate the ITG3200

    // Magnetometer Init
    if(!Magn.init(HMC5883L_Address, &hi2c1))
        return false;
    HAL_Delay(100);
    if(Magn.SetScale(1300) != 0)
        return false;
    HAL_Delay(100);

    // Set the measurement mode to Continuous
    if(Magn.SetMeasurementMode(Measurement_Continuous) != 0)
        return false;
    return true;
}


void
AHRS_Init_Position() {
    Madgwick.begin(float(samplingFrequency));

    // Get the first Sensor data
    while(!Acc.getInterruptSource(7)) {}
    Acc.get_Gxyz(&AHRSvalues[0]);
    while(!Gyro.isRawDataReadyOn()) {}
    Gyro.readGyro(&AHRSvalues[3]);
    while(!Magn.isDataReady()) {}
    Magn.ReadScaledAxis(&AHRSvalues[6]);

    // Initial estimate of the attitude (assumed a static sensor !)
    for(int i=0; i<10000; i++) { // ~13us per Madgwick.update() with NUCLEO-F411RE
        Madgwick.update(AHRSvalues[3], AHRSvalues[4], AHRSvalues[5], // Gyro in degrees/sec
                        AHRSvalues[0], AHRSvalues[1], AHRSvalues[2], // Acc
                        AHRSvalues[6], AHRSvalues[7], AHRSvalues[8]);// Mag
    }
}


// This function handles TIM2 global interrupt.
void
TIM2_IRQHandler(void) {
    if(pLeftControlledMotor)
        pLeftControlledMotor->Update();
    if(bAHRSpresent) {
        Acc.get_Gxyz(&AHRSvalues[0]);
        Gyro.readGyro(&AHRSvalues[3], &AHRSvalues[4], &AHRSvalues[5]);
        Magn.ReadScaledAxis(&AHRSvalues[6]);
        for(int i=0; i<3; i++) // 13us per call...
            Madgwick.update(AHRSvalues[3], AHRSvalues[4], AHRSvalues[5],
                            AHRSvalues[0], AHRSvalues[1], AHRSvalues[2],
                            AHRSvalues[6], AHRSvalues[7], AHRSvalues[8]);
    }
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    //HAL_TIM_IRQHandler(&htim2);
}


//  * @brief This function handles EXTI line[15:10] interrupts.
void
EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(B1_Pin);
}


void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == B1_Pin) {
        bRun = !bRun;
    }
}


// Tx Transfer completed callback
// UartHandle: UART handle.
void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
    (void)UartHandle;
    bTxUartReady = true;
}


// Rx Transfer completed callback
// UartHandle: UART handle
void
HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
    (void)UartHandle;
    bRxUartReady = true;
}


// This function handles DMA TX interrupt request.
void
USART2_DMA_TX_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart2.hdmatx);
}


// This function handles DMA RX interrupt request.
void
USART2_DMA_RX_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart2.hdmarx);
}


// This function handles USART2 interrupt request.
void
USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart2);
}


// UART error callbacks
// UartHandle: UART handle
void
HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
    (void)UartHandle;
    Error_Handler();
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
