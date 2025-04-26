#pragma once

#include "stm32f446xx.h"

void SysTick_Init();
void delay_ms(uint32_t ms);
uint32_t getMillis();

extern "C" void SysTick_Handler(void);