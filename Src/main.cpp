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


//==========================================
// TIM5 GPIO Configuration (Sonar)
//==========================================
// PA0 (CN7 - 28)  ------> TIM3_CH1 (Pulse)
// PA1 (CN7 - 30)  ------> TIM3_CH2 (Echo)


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
#include "i2c.h"
#include "uart.h"
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


TIM_HandleTypeDef  hSamplingTimer;     // Periodic Sampling Timer
TIM_HandleTypeDef  hLeftEncoderTimer;  // Left Motor Encoder Timer
TIM_HandleTypeDef  hRightEncoderTimer; // Right Motor Encoder Timer
TIM_HandleTypeDef  hPwmTimer;          // Dc Motors PWM (Speed control)
TIM_HandleTypeDef  hSonarEchoTimer;    // To Measure the Radar Echo Pulse Duration
TIM_HandleTypeDef  hSonarPulseTimer;   // To Generate the Radar Trigger Pulse

I2C_HandleTypeDef  hi2c2;

unsigned int baudRate = 9600;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart2_tx;
DMA_HandleTypeDef  hdma_usart2_rx;


double periodicClockFrequency = 10.0e6;  // 10MHz
double pwmClockFrequency      = 20.0e3;  // 10KHz
double sonarClockFrequency    = 10.0e6;  // 10MHz (100ns period)
double sonarPulseDelay        = 10.0e-6; // in seconds
double sonarPulseWidth        = 10.0e-6; // in seconds
double soundSpeed             = 340.0;   // in m/s


//==================
// Private variables
//==================

Encoder*         pLeftEncoder          = nullptr;
Encoder*         pRightEncoder         = nullptr;
DcMotor*         pLeftMotor            = nullptr;
DcMotor*         pRightMotor           = nullptr;
ControlledMotor* pLeftControlledMotor  = nullptr;
ControlledMotor* pRightControlledMotor = nullptr;


ADXL345  Acc;      // 400KHz I2C Capable. Maximum Output Data Rate is 800 Hz
ITG3200  Gyro;     // 400KHz I2C Capable
HMC5883L Magn;     // 400KHz I2C Capable, left at the default 15Hz data Rate
Madgwick Madgwick; // ~13us per Madgwick.update() with NUCLEO-F411RE

static float q0, q1, q2, q3;
static volatile float AHRSvalues[9];


uint32_t AHRSSamplingFrequency   = 400; // [Hz]
uint32_t motorSamplingFrequency  = 50;  // [Hz]
uint32_t sonarSamplingFrequency  = 5;   // [Hz]

uint32_t AHRSSamplingPulses  = uint32_t(periodicClockFrequency/AHRSSamplingFrequency +0.5); // [Hz]
uint32_t motorSamplingPulses = uint32_t(periodicClockFrequency/motorSamplingFrequency+0.5); // [Hz]
uint32_t sonarSamplingPulses = uint32_t(periodicClockFrequency/sonarSamplingFrequency+0.5); // [Hz]

bool bAHRSpresent  = false;
bool bSendAHRS     = false;
bool bSendMotors   = false;
bool bSendDistance = false;

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

// Captured Values for Echo Width Calculation
volatile uint32_t uwIC2Value1    = 0;
volatile uint32_t uwIC2Value2    = 0;
volatile uint32_t uwDiffCapture  = 0;
volatile uint16_t uhCaptureIndex = 0;

double x[3]; // Position
double v[3]; // Speed

//====================================
// Private function prototypes
//====================================
static void Init();
static void Loop();
static bool AHRS_Init();
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
    SerialPortInit();     // Initialize the Serial Communication Port (/dev/ttyACM0)

    LeftEncoderTimerInit();  // Initialize Left Motor Encoder
    RightEncoderTimerInit(); // Initialize Right Motor Encoder
    pLeftEncoder  = new Encoder(&hLeftEncoderTimer);
    pRightEncoder = new Encoder(&hRightEncoderTimer);

    PwmTimerInit(); // Initialize the Dc Motors
    // DcMotor(forwardPort, forwardPin, reversePort,  reversePin,
    //         pwmPort,  pwmPin, pwmTimer)
    pLeftMotor  = new DcMotor(GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9,
                              GPIOA, GPIO_PIN_6, &hPwmTimer, TIM_CHANNEL_1);
    pRightMotor = new DcMotor(GPIOC, GPIO_PIN_10, GPIOC, GPIO_PIN_11,
                              GPIOA, GPIO_PIN_7, &hPwmTimer, TIM_CHANNEL_2);

    // Initialize the Periodic Samplig Timer
    SamplingTimerInit(AHRSSamplingPulses, motorSamplingPulses, sonarSamplingPulses);

    I2C2_Init(); // Initialize the 10DOF Sensor (Wide power input range from 3 to 5 volts)
    bAHRSpresent = AHRS_Init();

    // Initialize Motor Controllers
    pLeftControlledMotor  = new ControlledMotor(pLeftMotor,  pLeftEncoder,  motorSamplingFrequency);
    pRightControlledMotor = new ControlledMotor(pRightMotor, pRightEncoder, motorSamplingFrequency);

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
    HAL_StatusTypeDef uartStatus;
    uartStatus = HAL_UART_Receive_DMA(&huart2, &inChar, 1);
    if(uartStatus == HAL_ERROR)
        Error_Handler();
    bRxUartReady = uartStatus == HAL_OK;

    strcpy((char *)txBuffer, "Buggy Ready\n");
    bConnected = false;
    while(!bConnected) {
        if(bRxComplete) {
            bRxComplete = false;
            ExecCommand();
        }
        if(bTxUartReady) {
            uartStatus = HAL_UART_Transmit_DMA(&huart2, txBuffer, strlen((char *)txBuffer));
            if(uartStatus == HAL_ERROR)
                Error_Handler();
            bTxUartReady = uartStatus != HAL_OK;
        }
        if(!bRxUartReady) {
            uartStatus = HAL_UART_Receive_DMA(&huart2, &inChar, 1);
            if(uartStatus == HAL_ERROR)
                Error_Handler();
            bRxUartReady = uartStatus == HAL_OK;
        }
    } //  while(!bConnected)
}


///=======================================================================
///                                Main Loop
///=======================================================================
static void
Loop() {
    int len;
    HAL_StatusTypeDef uartStatus;
    while(true) {
        // Wait Until Connected With Remote
        if(!bConnected) { // Asyncronously Changed Via Interrupts
            if(pLeftControlledMotor)
                pLeftControlledMotor->Stop();
            if(pRightControlledMotor)
                pRightControlledMotor->Stop();
            Wait4Connection(); // Blocking Call...
        }

        if(!bRxUartReady) {
            uartStatus = HAL_UART_Receive_DMA(&huart2, &inChar, 1);
            if(uartStatus == HAL_ERROR)
                Error_Handler();
            bRxUartReady = uartStatus == HAL_OK;
        }

        if(bRxComplete) { // A complete command has been received
            bRxComplete = false;
            ExecCommand();
        }

        // Transmit new Data only if the previous ones were sent
        if(bTxUartReady) {
            len = 0;
            if(bSendAHRS) {
                bSendAHRS = false;
                Madgwick.getRotation(&q0, &q1, &q2, &q3);
                len = sprintf((char *)txBuffer, "A,%d,%d,%d,%d",
                              int(q0*1000.0), int(q1*1000.0), int(q2*1000.0), int(q3*1000.0)
                              );
            }
            if(bSendMotors) {
                bSendMotors = false;
                len += sprintf((char *)&txBuffer[len], ",M,%d,%ld,%d,%ld",
                               int(pLeftControlledMotor->currentSpeed*100.0),
                               long(pLeftControlledMotor->getTotalMove()),
                               int(pRightControlledMotor->currentSpeed*100.0),
                               long(pRightControlledMotor->getTotalMove()));
            }
            if(bSendDistance) {
                bSendDistance = false;
                double distance = 0.5* soundSpeed *(double(uwDiffCapture)/sonarClockFrequency); // [m]
                len += sprintf((char *)&txBuffer[len], ",D,%d",
                               int(distance*100.0));
            }
            if(len > 0) {
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                len += sprintf((char *)&txBuffer[len], ",T,%lu\n",
                               static_cast<unsigned long>(HAL_GetTick()));
                bTxUartReady = false; // Asyncronously Changed Via Interrupts
                uartStatus = HAL_UART_Transmit_DMA(&huart2, txBuffer, len);
                if(uartStatus == HAL_ERROR)
                    Error_Handler();
                bTxUartReady = uartStatus != HAL_OK;
            }
        } // if(bTxUartReady)

    } // while(true)
} // Loop()
///=======================================================================
///                                 End Loop
///=======================================================================


bool
AHRS_Init() {
// Accelerator Init
    if(!Acc.init(ADXL345_ADDR_ALT_LOW, &hi2c2))
        return false;
// Gyroscope Init
    if(!Gyro.init(ITG3200_ADDR_AD0_LOW, &hi2c2))
        return false;
    HAL_Delay(100);
    Gyro.zeroCalibrate(1200); // calibrate the ITG3200
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
    Madgwick.begin(float(AHRSSamplingFrequency));
// Get the first Sensor data
    while(!Acc.getInterruptSource(7)) {}
    Acc.get_Gxyz(&AHRSvalues[0]);
    while(!Gyro.isRawDataReadyOn()) {}
    Gyro.readGyro(&AHRSvalues[3]);
    while(!Magn.isDataReady()) {}
    Magn.ReadScaledAxis(&AHRSvalues[6]);
// Initial estimate of the attitude (assuming a static sensor !)
    for(int i=0; i<20000; i++) { // ~13us per Madgwick.update() with NUCLEO-F411RE
        Madgwick.update(AHRSvalues[3], AHRSvalues[4], AHRSvalues[5], // Gyro in degrees/sec
                        AHRSvalues[0], AHRSvalues[1], AHRSvalues[2], // Acc
                        AHRSvalues[6], AHRSvalues[7], AHRSvalues[8]);// Mag
    }
    return true;
}


void
ExecCommand() {
    if(command[0] == 'K') { // Keep Alive
        bConnected = true;
        return;
    }
    if(command[0] == 'G') { // Go
        Acc.calibrate();
        x[0] = x[1] = x[2] = 0.0;
        v[0] = v[1] = v[2] = 0.0;
        return;
    }
    else if(command[0] == 'H') { // Halt
        pLeftControlledMotor->setTargetSpeed(0.0);
        pRightControlledMotor->setTargetSpeed(0.0);
        return;
    }

    int commandLen = strlen((const char*)(&command[0]));
    if(commandLen < 3)
        return; // At least 3 characters are needed

    ControlledMotor* pDestinationMotor;
    if(command[0] == 'L') // Left Motor commands
        pDestinationMotor = pLeftControlledMotor;
    else if(command[0] == 'R') // Right Motor commands
        pDestinationMotor = pRightControlledMotor;
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


// Initializes the Global MSP.
void
HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
}


// To handle the TIM2 (Periodic Sampler) interrupt.
void
TIM2_IRQHandler(void) { // Defined in file "startup_stm32f411xe.s"
    // Will call ... HAL_TIM_OC_DelayElapsedCallback(&hSamplingTimer)
    HAL_TIM_IRQHandler(&hSamplingTimer);
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
            uhCaptureIndex = 0;
            LL_TIM_IC_SetPolarity(TIM5, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
            LL_TIM_EnableCounter(hSonarPulseTimer.Instance);
            if(bOldConnectionStatus != bNewConnectionStatus)
                bConnected = false;
            else
                bOldConnectionStatus = !bOldConnectionStatus;
        }
    } // htim->Instance == hSamplingTimer.Instance
}


// To handle the TIM5 (Sonar Echo) interrupt.
void
TIM5_IRQHandler(void) {
    // Will call ... HAL_TIM_IC_CaptureCallback(&hSonarEchoTimer)
    HAL_TIM_IRQHandler(&hSonarEchoTimer);
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
        if(uwIC2Value2 >= uwIC2Value1) {
            uwDiffCapture = (uwIC2Value2 - uwIC2Value1);
        }
        else if(uwIC2Value2 < uwIC2Value1) { // 0xFFFFFFFF is max TIM5_CCRx value
            uwDiffCapture = ((0xFFFFFFFF - uwIC2Value1) + uwIC2Value2) + 1;
        }
        uhCaptureIndex = 0;
        bSendDistance = true;
    }
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
    HAL_StatusTypeDef uartStatus;
    bOldConnectionStatus = bNewConnectionStatus;
    if(inChar == '\n') {
        int i = 0;
        int index = rxBufferStart;
        while(index != rxBufferEnd) {
            command[i++] = rxBuffer[index++];
            index = index % rxBufferSize;
        }
        command[i] = 0;
        rxBufferEnd++;
        rxBufferEnd = rxBufferEnd % rxBufferSize;
        rxBufferStart = rxBufferEnd;
        bRxComplete = true;
        uartStatus = HAL_UART_Receive_DMA(&huart2, &inChar, 1);
        bRxUartReady = uartStatus == HAL_OK;
    }
    else {
        rxBuffer[rxBufferEnd++] = inChar;
        rxBufferEnd = rxBufferEnd % rxBufferSize;
        uartStatus = HAL_UART_Receive_DMA(&huart2, &inChar, 1);
        bRxUartReady = uartStatus == HAL_OK;
    }
    if(uartStatus == HAL_ERROR)
        Error_Handler();
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
