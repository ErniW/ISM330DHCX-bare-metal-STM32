#include "uart.h"

#define APB1_FREQ 16000000

extern "C" {
    int __io_putchar(int ch);
}

int __io_putchar(int ch){
    tx_send(ch);
    return ch;
}

void tx_init(){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    GPIOA->MODER |= PA2_AF;
    GPIOA->AFR[0] |= PA2_AF_USART2_TX;

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = (APB1_FREQ + (BAUDRATE/2U))/BAUDRATE;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

void tx_send(char c){
    while(!(USART2->SR & USART_SR_TXE)){};
    USART2->DR = (c & 0xFF);
};