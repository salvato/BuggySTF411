#include "utility.h"

void
Error_Handler(void) {
    while(true) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(50);
    }
}

