/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


#include "main.h"
#include "utility.h"


extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;



// Initializes the Global MSP.
void
HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
}


/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used
* @param hi2c: I2C handle pointer
* @retval None
*/
void
HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hi2c->Instance==I2C1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        // I2C1 GPIO Configuration
        // PB8     ------> I2C1_SCL
        // PB9     ------> I2C1_SDA
        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}


/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used
* @param hi2c: I2C handle pointer
* @retval None
*/
void
HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
    if(hi2c->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_DISABLE();
        // I2C1 GPIO Configuration
        // PB6     ------> I2C1_SCL
        // PB7     ------> I2C1_SDA
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
    }
}


/**
* @brief TIM_Encoder MSP De-Initialization
* This function freeze the hardware resources used
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void
HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder) {
    if(htim_encoder->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_DISABLE();
        // TIM1 GPIO Configuration
        // PA8     ------> TIM1_CH1
        // PA9     ------> TIM1_CH2
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9);
    }
    else if(htim_encoder->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_DISABLE();
        // TIM4 GPIO Configuration
        // PB6     ------> TIM4_CH1
        // PB7     ------> TIM4_CH2
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    }
}


/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void
HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim_base->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
    else if(htim_base->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // TIM3 GPIO Configuration
        // PA6     ------> TIM3_CH1
        // PA7     ------> TIM3_CH2
        GPIO_InitStruct.Pin       = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}


/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void
HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
    if(htim_base->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
    }
    else if(htim_base->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_DISABLE();
        // TIM3 GPIO Configuration
        // PA6     ------> TIM3_CH1
        // PA7     ------> TIM3_CH2
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6 | GPIO_PIN_7);
    }
}


/**
* @brief UART MSP Initialization
* This function configures the hardware resources used
* @param huart: UART handle pointer
* @retval None
*/
void
HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // USART2 GPIO Configuration
        // PA2     ------> USART2_TX
        // PA3     ------> USART2_RX
        GPIO_InitStruct.Pin       = USART_TX_Pin|USART_RX_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        // USART2 DMA Init
        // USART2_TX Init
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
        if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmatx, hdma_usart2_tx);
        // USART2_RX Init
        hdma_usart2_rx.Instance = DMA1_Stream5;
        hdma_usart2_rx.Init.Channel             = DMA_CHANNEL_4;
        hdma_usart2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_usart2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart2_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart2_rx.Init.Mode                = DMA_NORMAL;
        hdma_usart2_rx.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_usart2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);
    }
}


/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used
* @param huart: UART handle pointer
* @retval None
*/
void
HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
    if(huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_DISABLE();
        // USART2 GPIO Configuration
        // PA2     ------> USART2_TX
        // PA3     ------> USART2_RX
        HAL_GPIO_DeInit(GPIOA, USART_TX_Pin | USART_RX_Pin);
        HAL_DMA_DeInit(huart->hdmatx);
        HAL_DMA_DeInit(huart->hdmarx);
    }
}
