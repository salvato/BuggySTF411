/*
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


//===================================
// I2C2 GPIO Configuration
//===================================
// VIN  (CN7  16)    ------> +3.3V
// GND  (CN7  20)    ------> GND
// PB10 (CN10 25)    ------> I2C2_SCL
// PB3  (CN10 31)    ------> I2C2_SDA


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

//====================================
// Used Peripherals:
// Timers:
//     TIM1 ---> Encoder (Left)
//     TIM2 ---> Periodic Interrupt
//     TIM3 ---> PWM (Motors)
//     TIM4 ---> Encoder (Right)
//     TIM9 ---> Ultrasound Sensor
//====================================
*/

#include "main.h"
#include "tim.h"
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


#define USART2_FORCE_RESET()   __HAL_RCC_USART2_FORCE_RESET()
#define USARt2_RELEASE_RESET() __HAL_RCC_USART2_RELEASE_RESET()


TIM_HandleTypeDef hLeftEncodertimer;
TIM_HandleTypeDef hSamplingTimer;
TIM_HandleTypeDef hPwmTimer;
TIM_HandleTypeDef hRightEncodertimer;
TIM_HandleTypeDef hSonarEchoTimer; // To Measure the Radar Echo Pulse Duration
TIM_HandleTypeDef hSonarPulseTimer; // To Measure the Radar Echo Pulse Duration


double periodicCounterClock  = 10.0e6;  // 10MHz
double sonarClockFrequency   = 10.0e6;  // 10MHz (100ns period)
double sonarPulseDelay       = 10.0e-6; // in seconds
double sonarPulseWidth       = 10.0e-6; // in seconds


//==================
// Private variables
//==================

UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart2_tx;
DMA_HandleTypeDef  hdma_usart2_rx;
I2C_HandleTypeDef  hi2c2;


Encoder*         pLeftEncoder          = nullptr;
Encoder*         pRightEncoder         = nullptr;
DcMotor*         pLeftMotor            = nullptr;
DcMotor*         pRightMotor           = nullptr;
ControlledMotor* pLeftControlledMotor  = nullptr;
ControlledMotor* pRightControlledMotor = nullptr;


bool bAHRSpresent = false;
ADXL345  Acc;      // 400KHz I2C Capable. Maximum Output Data Rate is 800 Hz
ITG3200  Gyro;     // 400KHz I2C Capable
HMC5883L Magn;     // 400KHz I2C Capable, left at the default 15Hz data Rate
Madgwick Madgwick; // ~13us per Madgwick.update() with NUCLEO-F411RE
static float q0, q1, q2, q3;
static volatile float AHRSvalues[9];


uint32_t AHRSSamplingFrequency   = 400; // [Hz]
uint32_t motorSamplingFrequency  = 50;  // [Hz]
uint32_t sonarSamplingFrequency  = 5;   // [Hz]

uint32_t AHRSSamplingPulses  = uint32_t(periodicCounterClock/AHRSSamplingFrequency +0.5);  // [Hz]
uint32_t motorSamplingPulses = uint32_t(periodicCounterClock/motorSamplingFrequency+0.5); // [Hz]
uint32_t sonarSamplingPulses = uint32_t(periodicCounterClock/sonarSamplingFrequency+0.5); // [Hz]

bool bSendAHRS    = false;
bool bSendMotors  = false;

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

volatile bool bConnected           = false;
volatile bool bNewConnectionStatus = false;
volatile bool bOldConnectionStatus = bNewConnectionStatus;

/* Captured Values */
volatile uint32_t uwIC2Value1    = 0;
volatile uint32_t uwIC2Value2    = 0;
volatile uint32_t uwDiffCapture  = 0;
/* Capture index */
volatile uint16_t uhCaptureIndex = 0;
/* Frequency Value */
volatile uint32_t uwFrequency    = 0;
volatile double uwMeasuredDelay;
volatile double uwMeasuredPulseLength;


//====================================
// Private function prototypes
//====================================
static void Init();
static void Loop();
static void I2C2_Init(void);
static void SerialPort_Init(void);
static bool AHRS_Init();
static void AHRS_Init_Position();
static void ExecCommand();
static void Wait4Connection();


int
main(void) {
    Init();
    Loop();
}


static void
Init() {
    HAL_Init();           // Initialize the HAL Library
    SystemClock_Config(); // Initialize System Clock
    GPIO_Init();          // Initialize On Board Peripherals
    SerialPort_Init();    // Initialize the Serial Communication Port (/dev/ttyACM0)

    LeftEncoderTimerInit(); // Initialize Left Motor Encoder
    pLeftEncoder = new Encoder(&hLeftEncodertimer);
    RightEncoderTimerInit(); // Initialize Right Motor Encoder
    pRightEncoder = new Encoder(&hRightEncodertimer);

    PwmTimerInit(); // Initialize the Dc Motors
    // DcMotor(forwardPort, forwardPin, reversePort,  reversePin,
    //         pwmPort,  pwmPin, pwmTimer)
    pLeftMotor = new DcMotor(GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9,
                             GPIOA, GPIO_PIN_6, &hPwmTimer, TIM_CHANNEL_1);
    pRightMotor = new DcMotor(GPIOC, GPIO_PIN_10, GPIOC, GPIO_PIN_11,
                              GPIOA, GPIO_PIN_7, &hPwmTimer, TIM_CHANNEL_2);

    // Initialize the Periodic Samplig Timer
    SamplingTimerInit(AHRSSamplingPulses, motorSamplingPulses, sonarSamplingPulses);

    I2C2_Init(); // Initialize the 10DOF Sensor
    bAHRSpresent = AHRS_Init();
    if(bAHRSpresent) // 10DOF Sensor Position Initialization
        AHRS_Init_Position();

    // Initialize Motor Controllers
    pLeftControlledMotor  = new ControlledMotor(pLeftMotor,  pLeftEncoder,  motorSamplingFrequency);
//    pRightControlledMotor = new ControlledMotor(pRightMotor, pRightEncoder, motorSamplingFrequency);

    SonarEchoTimerInit();
    SonarPulseTimerInit();

    // Start the Periodic Samplig Timer
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    // Start the Periodic Sampling of AHRS, Motors and Sonar
    if(HAL_TIM_OC_Start_IT(&hSamplingTimer, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    if(HAL_TIM_OC_Start_IT(&hSamplingTimer, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if(HAL_TIM_OC_Start_IT(&hSamplingTimer, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable and set Button EXTI Interrupt

    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}


void
Wait4Connection() {
    rxBufferStart = 0;
    rxBufferEnd   = 0;
    bTxUartReady  = true;
    bRxComplete   = false;
    bRxUartReady  = false;

    // Ensure an Empty UART Buffer
    HAL_UART_AbortReceive(&huart2);
    while(HAL_UART_Receive(&huart2, &inChar, 1, 100) == HAL_OK) ;

    // Then prepare to receive commands
    if(HAL_UART_Receive_DMA(&huart2, &inChar, 1) != HAL_OK)
        Error_Handler();

    //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    strcpy((char *)txBuffer, "Buggy Ready\n");
    bConnected = false;
    while(!bConnected) {
        if(bRxUartReady) {
            bRxUartReady = false;
            if(HAL_UART_Receive_DMA(&huart2, &inChar, 1) != HAL_OK)
                Error_Handler();
        }
        if(bRxComplete) {
            bRxComplete = false;
            ExecCommand();
            if(HAL_UART_Receive_DMA(&huart2, &inChar, 1) != HAL_OK)
                Error_Handler();
        }
        if(bTxUartReady) {
            bTxUartReady = false;
            if(HAL_UART_Transmit_DMA(&huart2, txBuffer, strlen((char *)txBuffer)) != HAL_OK)
                Error_Handler();
        }
    } //  while(!bConnected)
    //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}


///=======================================================================
///                                Main Loop
///=======================================================================
static void
Loop() {
    int len;
    while(true) {
        // Wait Until Connected With Remote
        if(!bConnected) { // Asyncronously Changed Via Interrupts
            if(pLeftControlledMotor)
                pLeftControlledMotor->Stop();
            if(pRightControlledMotor)
                pRightControlledMotor->Stop();
            Wait4Connection(); // Blocking Call...
        }

        // Transmit new Data only if the previous ones were sent
        if(bTxUartReady) {
            len = 0;
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
                bTxUartReady = false; // Asyncronously Changed Via Interrupts
                if(HAL_UART_Transmit_DMA(&huart2, txBuffer, len) != HAL_OK)
                    Error_Handler();
            }
        } // if(bTxUartReady)

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

    } // while(true)
} // Loop()
///=======================================================================
///                                 End Loop
///=======================================================================


static void
I2C2_Init(void) {
    __HAL_RCC_I2C2_CLK_ENABLE();
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed      = I2C_SPEEDCLOCK;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
}


// @brief I2C MSP Initialization
// This function configures the hardware resources used
// @param hi2c: I2C handle pointer
// @retval None
void
HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    if(hi2c->Instance==I2C2) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        // I2C2 GPIO Configuration
        // PB10  ------> I2C2_SCL
        // PB3   ------> I2C2_SDA
        GPIO_InitStruct.Pin       = GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = GPIO_PIN_3;
        GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C2_CLK_ENABLE();
    }
}


// @brief I2C MSP De-Initialization
// This function freeze the hardware resources used
// @param hi2c: I2C handle pointer
// @retval None

void
HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
    if(hi2c->Instance == I2C2) {
        __HAL_RCC_I2C2_CLK_DISABLE();
        // I2C1 GPIO Configuration
        // PB10  ------> I2C1_SCL
        // PB3   ------> I2C1_SDA
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_10);
    }
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

// Configure the NVIC for DMA ##########################################
// NVIC configuration for DMA Tx transfer complete interrupt
    HAL_NVIC_SetPriority(USART2_DMA_TX_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART2_DMA_TX_IRQn);

// NVIC configuration for DMA Rx transfer complete interrupt
    HAL_NVIC_SetPriority(USART2_DMA_RX_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART2_DMA_RX_IRQn);

// NVIC configuration for USART TC interrupt
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}


bool
AHRS_Init() {
// Accelerator Init
    if(!Acc.init(ADXL345_ADDR_ALT_LOW, &hi2c2))
        return false;
    Acc.setRangeSetting(2); // +/- 2g. Possible values are: 2g, 4g, 8g, 16g
// Gyroscope Init
    if(!Gyro.init(ITG3200_ADDR_AD0_LOW, &hi2c2))
        return false;
    HAL_Delay(100);
    Gyro.zeroCalibrate(600); // calibrate the ITG3200
// Magnetometer Init
    if(!Magn.init(HMC5883L_Address, &hi2c2))
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
    Madgwick.begin(float(AHRSSamplingFrequency));
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
    if(command[0] == 'K') { // Keep Alive
        bConnected = true;
        return;
    }
    if(command[0] == 'G') { // Go
        return;
    }
    else if(command[0] == 'H') { // Halt
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
        return; // <<<<<<<<<<<========================= No Right Motor at Present
        //pDestinationMotor = pRightControlledMotor;
    else
        return; // Command Error !

    if(command[1] == 's') { // New Speed
        int newValue;
        sscanf((const char*)(&command[2]), "%d", &newValue);
        pDestinationMotor->setTargetSpeed(newValue/100.0);

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


// Called when the Sonar Echo Signal Change Level
void
HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if(uhCaptureIndex == 0) { // Get the 1st Input Capture value
        // Select the next edge of the active transition on the TI2 channel: falling edge
        LL_TIM_IC_SetPolarity(TIM5, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);
        uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        uhCaptureIndex = 1;
    }
    else if(uhCaptureIndex == 1) { // Get the 2nd Input Capture value
        // Select the next edge of the active transition on the TI2 channel: rising edge
        LL_TIM_IC_SetPolarity(TIM5, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
        uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        /// Echo duration computation
        if(uwIC2Value2 > uwIC2Value1) {
            uwDiffCapture = (uwIC2Value2 - uwIC2Value1);
        }
        else if(uwIC2Value2 < uwIC2Value1) { // 0xFFFF is max TIM5_CCRx value
            uwDiffCapture = ((0xFFFF - uwIC2Value1) + uwIC2Value2) + 1;
        }
        else { // If capture values are equal, we have reached the limit of frequency measures
            Error_Handler();
        }
        uhCaptureIndex = 0;
    }
}


// To handle the TIM2 (Periodic Sampler) interrupt.
void
TIM2_IRQHandler(void) { // Defined in file "startup_stm32f411xe.s"
    // Will call ... HAL_TIM_OC_DelayElapsedCallback(&hSamplingTimer)
    HAL_TIM_IRQHandler(&hSamplingTimer);
}



// To handle the TIM5 (Sonar Echo) interrupt.
void
TIM5_IRQHandler(void) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    // Will call ... HAL_TIM_IC_CaptureCallback(&hSonarEchoTimer)
    HAL_TIM_IRQHandler(&hSonarEchoTimer);
}


 // To Restart Periodic Timers
void
HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == hSamplingTimer.Instance) {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) { // Is it time to Update AHRS Data ?
            htim->Instance->CCR4 += AHRSSamplingPulses;
            if(bAHRSpresent) {
                Acc.get_Gxyz(&AHRSvalues[0]);
                Gyro.readGyro(&AHRSvalues[3], &AHRSvalues[4], &AHRSvalues[5]);
                Magn.ReadScaledAxis(&AHRSvalues[6]);
                for(int i=0; i<3; i++) // ~13us per call...
                    Madgwick.update(AHRSvalues[3], AHRSvalues[4], AHRSvalues[5],
                                    AHRSvalues[0], AHRSvalues[1], AHRSvalues[2],
                                    AHRSvalues[6], AHRSvalues[7], AHRSvalues[8]);
                bSendAHRS = true;
            }
            //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) { // Is it time to Update Motors Data ?
            htim->Instance->CCR2 += motorSamplingPulses;
            if(pLeftControlledMotor) {
                pLeftControlledMotor->Update();
                bSendMotors = true;
            }
            if(pRightControlledMotor) {
                pRightControlledMotor->Update();
                bSendMotors = true;
            }
        }
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) { // Is it time to Update Sonar Data ? 1Hz
            htim->Instance->CCR3 += sonarSamplingPulses;
            // Enable counter.
            // The counter will stop automatically at the next update event (UEV).
            LL_TIM_EnableCounter(hSonarPulseTimer.Instance);
            if(bOldConnectionStatus != bNewConnectionStatus)
                bConnected = false;
            else
                bOldConnectionStatus = !bOldConnectionStatus;
        }
    } // htim->Instance == hSamplingTimer.Instance
}


// To handle the EXTI line[15:10] interrupts.
void
EXTI15_10_IRQHandler(void) { // defined in file "startup_stm32f411xe.s"
    HAL_GPIO_EXTI_IRQHandler(B1_Pin);
}


// EXTI line[15:10] interrupts handler
void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == B1_Pin) {

    }
}


// Tx Transfer completed callback
void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
    (void)UartHandle;
    bTxUartReady = true;
}


// Rx Transfer completed callback
void
HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
    (void)UartHandle;
    bOldConnectionStatus = bNewConnectionStatus;
    if(inChar == '\n') {
        int i = 0;
        int index = rxBufferStart;
        while((index % rxBufferSize) != rxBufferEnd)
            command[i++] = rxBuffer[index++];
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


// To handle DMA TX interrupt request.
void
USART2_DMA_TX_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart2.hdmatx);
}


// To handle DMA RX interrupt request.
void
USART2_DMA_RX_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart2.hdmarx);
}


// To handle USART2 interrupt request.
void
USART2_IRQHandler(void) { // defined in file "startup_stm32f411xe.s"
    HAL_UART_IRQHandler(&huart2);
}


void
HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
    (void)UartHandle;
    Error_Handler();
}


#ifdef  USE_FULL_ASSERT
void
assert_failed(uint8_t *file, uint32_t line) {
    sprintf((char *)txBuffer, "%s - line: %d", file, line);
    HAL_UART_Transmit(&huart2, txBuffer, strlen((char*)txBuffer), 1000);
    while(true) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(200);
    }
}
#endif // USE_FULL_ASSERT
