
#include "main.h"
#include "stm32f4xx_it.h"

extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;


//*****************************************************************************
//           Cortex-M4 Processor Interruption and Exception Handlers          *
//*****************************************************************************

//  * @brief This function handles Non maskable interrupt.
void
NMI_Handler(void) {
}


//  * @brief This function handles Hard fault interrupt.
void
HardFault_Handler(void) {
    while(1) {
    }
}


//  * @brief This function handles Memory management fault.
void
MemManage_Handler(void) {
    while(1) {
    }
}


//  * @brief This function handles Pre-fetch fault, memory access fault.
void
BusFault_Handler(void) {
    while(1) {
    }
}


//  * @brief This function handles Undefined instruction or illegal state.
void
UsageFault_Handler(void) {
    while(1) {
    }
}


//  * @brief This function handles System service call via SWI instruction.
void
SVC_Handler(void) {
}


//  * @brief This function handles Debug monitor.
void
DebugMon_Handler(void) {
}


//  * @brief This function handles Pendable request for system service.
void
PendSV_Handler(void) {
}


//  * @brief This function handles System tick timer.
void
SysTick_Handler(void) {
    HAL_IncTick();
}


//****************************************************************************
// STM32F4xx Peripheral Interrupt Handlers
// Add here the Interrupt Handlers for the used peripherals.
// For the available peripheral interrupt handler names,
// please refer to the startup file (startup_stm32f4xx.s).
//****************************************************************************


//  * @brief This function handles DMA1 stream5 global interrupt.
void
DMA1_Stream5_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}


//  * @brief This function handles DMA1 stream6 global interrupt.
void
DMA1_Stream6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}


//  * @brief This function handles TIM2 global interrupt.
void
TIM2_IRQHandler(void) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_TIM_IRQHandler(&htim2);
}
