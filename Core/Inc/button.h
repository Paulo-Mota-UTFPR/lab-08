#ifndef BUTTON_H__
#define BUTTON_H__

#include "stm32f4xx_hal.h"

uint16_t button(GPIO_TypeDef* ui32Port, uint16_t ui16Pin, uint16_t active_state);
uint16_t button_release(GPIO_TypeDef* ui32Port, uint16_t ui16Pin, uint16_t active_state);

#endif