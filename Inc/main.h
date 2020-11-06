#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define B1_Pin             GPIO_PIN_13
#define B1_GPIO_Port       GPIOC

#define USART_TX_Pin       GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin       GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA

#define USART_DMA_TX_IRQn       DMA1_Stream6_IRQn
#define USART_DMA_RX_IRQn       DMA1_Stream5_IRQn
#define USART_DMA_TX_IRQHandler DMA1_Stream6_IRQHandler
#define USART_DMA_RX_IRQHandler DMA1_Stream5_IRQHandler
#define USART_IRQn              USART2_IRQn

#define LD2_Pin            GPIO_PIN_5
#define LD2_GPIO_Port      GPIOA

void Error_Handler(void);

void TIM2_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void USART_DMA_RX_IRQHandler(void);
void USART_DMA_TX_IRQHandler(void);
void USART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif
