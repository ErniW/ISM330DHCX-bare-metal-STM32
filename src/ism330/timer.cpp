#include<stdint.h>
#include<stm32f446xx.h>
#include "i2c.h"

void timerInit(){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 &= ~TIM_CR1_CEN;

    TIM2->PSC = (SystemCoreClock / 1000000) - 1;
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CNT = 0;

    TIM2->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;     
}

void timerEnable(){
    TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t timerGetTime(){
    return TIM2->CNT;
}