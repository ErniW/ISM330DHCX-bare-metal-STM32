#pragma once

#include "stm32f446xx.h"

void SysTick_Init();
extern "C" void SysTick_Handler(void);
void delay_ms(uint32_t ms);