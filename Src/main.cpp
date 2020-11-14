// To configure QtCreator in order to Debug and Run programs
// on STM32F boards see:
//
// https://github.com/nlhans/qt-baremetal
// https://electronics.stackexchange.com/questions/212018/debugging-an-arm-stm32-microcontroller-using-qt-creator
//
// To see the console output:
//      Open a terminal and digit: cat /dev/ttyACM0
//
// Motor Gear 9:1

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


#include "main.h"
#include "utility.h"
#include "encoder.h"
#include "dcmotor.h"
#include "PID_v1.h"
#include "controlledmotor.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"
#include "string.h" // for memset()
#include "stdio.h"


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

static float q0, q1, q2, q3;
static volatile float AHRSvalues[9];

TIM_HandleTypeDef samplingTimer; // Samplig Timer
uint32_t motorSlowingSampler    = 6;
uint32_t samplingFrequency      = 300;                     // Sampling Frequency [Hz]
uint32_t motorSamplingFrequency = 300/motorSlowingSampler; // [Hz]

uint32_t callsNumber  = 0;
uint32_t callsNumber2 = 0;

bool bSendAHRS    = false;
bool bSendMotors  = false;
bool bChangeSpeed = false;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart2_tx;
DMA_HandleTypeDef  hdma_usart2_rx;
int     rxBufferSize = 255;
uint8_t rxBuffer[255];
uint8_t txBuffer[255];
uint8_t command[255];
uint8_t inChar;

volatile int  rxBufferStart;
volatile int  rxBufferEnd;
volatile bool bTxUartReady;
volatile bool bRxUartReady;
volatile bool bRxComplete;

volatile bool bRun;


//====================================
// Private function prototypes
//====================================
static void Init();
static void Loop();
static void I2C1_Init(void);
static void SamplingTimer_init(void);
static void SerialPort_Init(void);

static bool Sensors_Init();
static void AHRS_Init_Position();
static void ExecCommand();
static void Wait4Connection();


int
main(void) {
    Init();
    Wait4Connection();
    Loop();
}


static void
Init() {
// Initialize the HAL Library
    HAL_Init();
// Initialize System Clock
    SystemClock_Config();
// Initialize On Board Peripherals
    GPIO_Init();
// Initialize the Serial Communication Port (/dev/ttyACM0)
    SerialPort_Init();
// Initialize Left Motor and relative Encoder
    leftEncoder.init();
    leftEncoder.start();
    leftMotor.init();
// Initialize Right Motor and relative Encoder
    rightEncoder.init();
    rightEncoder.start();
    rightMotor.init();
// Initialize the Periodic Samplig Timer
    SamplingTimer_init();
// 10DOF Sensor Initialization
    I2C1_Init();
    bAHRSpresent = Sensors_Init();
// 10DOF Sensor Position Initialization
    if(bAHRSpresent)
        AHRS_Init_Position();
// Initialize Motor Controllers
    pLeftControlledMotor  = new ControlledMotor(&leftMotor,  &leftEncoder,  motorSamplingFrequency);
    pRightControlledMotor = new ControlledMotor(&rightMotor, &rightEncoder, motorSamplingFrequency);
// Start the Periodic Sampling of AHRS and Motors
    HAL_TIM_Base_Start_IT(&samplingTimer);
// Enable and set Button EXTI Interrupt
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


void
Wait4Connection() {
// Ensure an empty UART Buffer
    while(HAL_UART_Receive(&huart2, &inChar, 1, 10) == HAL_OK) ;
// DMA is programmed for reception in order to
// be sure DMA Rx is ready when counterpart will start transmitting
    rxBufferStart = 0;
    rxBufferEnd   = 0;
    bTxUartReady  = true;
    bRxComplete   = false;
    bRxUartReady  = false;
    if(HAL_UART_Receive_DMA(&huart2, &inChar, 1) != HAL_OK)
        Error_Handler();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    strcpy((char *)txBuffer, "Buggy Ready\n");
    bRun = false;
    while(!bRun) {
        if(bTxUartReady) {
            bTxUartReady = false;
            if(HAL_UART_Transmit_DMA(&huart2, txBuffer, strlen((char *)txBuffer)) != HAL_OK)
                Error_Handler();
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
        if(bRxUartReady) {
            bRxUartReady = false;
            if(HAL_UART_Receive_DMA(&huart2, &inChar, 1) != HAL_OK)
                Error_Handler();
        }
        if(bRxComplete) {
            bRxComplete = false;
            bRxUartReady = false;
            if(HAL_UART_Receive_DMA(&huart2, &inChar, 1) != HAL_OK)
                Error_Handler();
            if(command[0] == 'G') // Go Command Received !
                bRun = true;
        }
        HAL_Delay(100);
    } //  while(!bRun)
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}


//=======================================================================
//                                Main Loop
//=======================================================================
static void
Loop() {
    int ts = 0;
    pLeftControlledMotor->setTargetSpeed(2.0+ts);
    while(true) {
        if(bTxUartReady) {
            int len= 0;
            if(bSendAHRS) {
                bSendAHRS = false;
                Madgwick.getRotation(&q0, &q1, &q2, &q3);
                len = sprintf((char *)txBuffer, "A,%d,%d,%d,%d",
                              int(q0*1000.0), int(q1*1000.0), int(q2*1000.0), int(q3*1000.0));
            }
            if(bSendMotors) {
                bSendMotors = false;
                len += sprintf((char *)&txBuffer[len], ",M,%d,%ld",
                                  int(pLeftControlledMotor->currentSpeed*100.0),
                                  long(pLeftControlledMotor->getTotalMove()));
            }
            if(len > 0) {
                len += sprintf((char *)&txBuffer[len], ",T,%lu\n",
                               static_cast<unsigned long>(HAL_GetTick()));
                bTxUartReady = false;
                if(HAL_UART_Transmit_DMA(&huart2, txBuffer, strlen((char *)txBuffer)) != HAL_OK)
                    Error_Handler();
            }
        }
        if(bRxUartReady) {
            bRxUartReady = false;
            if(HAL_UART_Receive_DMA(&huart2, &inChar, 1) != HAL_OK)
                Error_Handler();
        }
        if(bRxComplete) {
            bRxComplete = false;
            ExecCommand();
            bRxUartReady = false;
            if(HAL_UART_Receive_DMA(&huart2, &inChar, 1) != HAL_OK)
                Error_Handler();
        }
        if(bChangeSpeed & bRun) {
            bChangeSpeed = false;
            ts = 1-ts;
            pLeftControlledMotor->setTargetSpeed(1.0+3*ts);

        }
    } // while(true)
}
//=======================================================================
//                                 End Loop
//=======================================================================


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
        Error_Handler();
    }
}


static void
SamplingTimer_init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
    memset(&sMasterConfig, 0, sizeof(sMasterConfig));

    __HAL_RCC_TIM2_CLK_ENABLE();

    // Time base configuration (based on 100 MHz CPU frequency)
    const uint32_t counterClock = SystemCoreClock/100;// 1MHz;
    // Prescaler value to have a 1MHz TIM2 Input Counter Clock
    uint32_t uwPrescalerValue = (uint32_t) (SystemCoreClock/counterClock)-1;

    samplingTimer.Instance = TIM2;
    samplingTimer.Init.Prescaler         = uwPrescalerValue;
    samplingTimer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    samplingTimer.Init.Period            = (counterClock/samplingFrequency)-1; // (Sampling period) / (Clock Period)
    samplingTimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; // tDTS=tCK_INT
    samplingTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if(HAL_TIM_Base_Init(&samplingTimer) != HAL_OK)
        Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if(HAL_TIM_ConfigClockSource(&samplingTimer, &sClockSourceConfig) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&samplingTimer, &sMasterConfig) != HAL_OK)
        Error_Handler();

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}


static void
SerialPort_Init(void) {
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
    if(HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();

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
    if(HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
        Error_Handler();
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
    if(HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
        Error_Handler();
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


void
ExecCommand() {
    if(command[0] == 'G') { // Go
        bRun = true;
        return;
    }
    else if(command[0] == 'H') { // Halt
        bRun = false;
        pLeftControlledMotor->setTargetSpeed(0.0);
        return;
    }

    int commandLen = strlen((const char*)(&command[0]));
    if(commandLen < 3)
        return; // At least 3 characters are needed

    ControlledMotor* pDestinationMotor;
    if(command[0] == 'L') // Left Motor commands
        pDestinationMotor = pLeftControlledMotor;
    else if(command[0] == 'R') // Right Motor commands
        return; // <<<<<<<<<<<<<<<<<<<<<<<<<<<========================= No Right Motor at Present
        //pDestinationMotor = pRightControlledMotor;
    else
        return; // Command Error !

    if(command[1] == 's') { // New Speed
        double newSpeed;
        sscanf((const char*)(&command[2]), "%lf", &newSpeed);
        pDestinationMotor->setTargetSpeed(newSpeed);

    }
    else if(command[1] == 'p') { // New Proportional PID value
        int newValue;
        sscanf((const char*)(&command[2]), "%d", &newValue);
        pDestinationMotor->setP(newValue);
    }
    else if(command[1] == 'i') { // New Integral PID value
        int newValue;
        sscanf((const char*)(&command[2]), "%d", &newValue);
        pDestinationMotor->setI(newValue);
    }
    else if(command[1] == 'd') { // New Differential PID value
        int newValue;
        sscanf((const char*)(&command[2]), "%d", &newValue);
        pDestinationMotor->setD(newValue);
    }
    return;
}


// To handle the TIM2 (Periodic Sampler) interrupt.
void
TIM2_IRQHandler(void) { // defined in file "startup_stm32f411xe.s"
    ++callsNumber2;
    callsNumber2 = callsNumber2 % 1200;
    if(callsNumber2)
        bChangeSpeed = true;

    ++callsNumber;
    callsNumber = callsNumber % motorSlowingSampler;
    if(!callsNumber) {
        if(pLeftControlledMotor)
            pLeftControlledMotor->Update();
        bSendMotors = true;
    }

    if(bAHRSpresent) {
        Acc.get_Gxyz(&AHRSvalues[0]);
        Gyro.readGyro(&AHRSvalues[3], &AHRSvalues[4], &AHRSvalues[5]);
        Magn.ReadScaledAxis(&AHRSvalues[6]);
        for(int i=0; i<3; i++) // 13us per call...
            Madgwick.update(AHRSvalues[3], AHRSvalues[4], AHRSvalues[5],
                            AHRSvalues[0], AHRSvalues[1], AHRSvalues[2],
                            AHRSvalues[6], AHRSvalues[7], AHRSvalues[8]);
        bSendAHRS = true;
    }
    __HAL_TIM_CLEAR_IT(&samplingTimer, uint32_t(TIM_IT_UPDATE));
    //HAL_TIM_IRQHandler(&samplingTimer);
}


// This function handles EXTI line[15:10] interrupts.
void
EXTI15_10_IRQHandler(void) { // defined in file "startup_stm32f411xe.s"
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
    if(inChar == '\n') {
        int i = 0;
        int index = rxBufferStart;
        while((index % rxBufferSize) != rxBufferEnd) {
            command[i++] = rxBuffer[index++];
        }
        command[i] = 0;
        rxBufferEnd++;
        rxBufferStart = rxBufferEnd;
        bRxComplete = true;
    }
    else {
        rxBuffer[rxBufferEnd] = inChar;
        rxBufferEnd++;
        rxBufferEnd = rxBufferEnd % rxBufferSize;
        bRxUartReady = true;
    }
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
USART2_IRQHandler(void) { // defined in file "startup_stm32f411xe.s"
    HAL_UART_IRQHandler(&huart2);
}


// UART error callbacks
// UartHandle: UART handle
void
HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
    (void)UartHandle;
    Error_Handler();
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
