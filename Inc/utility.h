#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define B1_Pin        GPIO_PIN_13
#define B1_GPIO_Port  GPIOC
#define LD2_Pin       GPIO_PIN_5
#define LD2_GPIO_Port GPIOA

void Error_Handler(void);
void SystemClock_Config(void);
void GPIO_Init(void);

#ifdef __cplusplus
}
#endif
