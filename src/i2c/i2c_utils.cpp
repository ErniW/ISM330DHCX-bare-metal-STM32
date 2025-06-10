#include "i2c.h"

void I2C1_manualRestart(){
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    for (volatile uint16_t i=0; i<1000; i++);
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    // I2C1->CR1 &= ~I2C_CR1_PE;
    RCC->APB1ENR &=~ RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &=~ (GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |= GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0;
    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;
    GPIOB->ODR |= GPIO_ODR_OD8 | GPIO_ODR_OD9;

    for (volatile uint16_t i=0; i<1000; i++);

    for (uint16_t i=0; i<9; i++) {
        if (GPIOB->IDR & GPIO_IDR_ID9) 
            break;

        GPIOB->ODR &= ~GPIO_ODR_OD8;
        for (volatile uint16_t j=0; j<1000; j++);
        GPIOB->ODR |= GPIO_ODR_OD8;
        for (volatile uint16_t j=0; j<1000; j++);
    }

    GPIOB->ODR &= ~GPIO_ODR_OD9;
    for (volatile uint16_t j=0; j<1000; j++);
    GPIOB->ODR |= GPIO_ODR_OD8;
    for (volatile uint16_t j=0; j<1000; j++);
    GPIOB->ODR |= GPIO_ODR_OD9;
    for (volatile uint16_t j=0; j<1000; j++);


    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1_gpioConfig();
 
}

void I2C1_gpioConfig(){
    GPIOB->MODER &=~ (GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |= PB8_AF_MODE | PB9_AF_MODE;

    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;

    GPIOB->PUPDR &=~ (GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);
    GPIOB->PUPDR |= PB8_PULLUP | PB9_PULLUP;

    GPIOB->AFR[1] |= PB8_AF4_I2C_SCL | PB9_AF4_I2C_SDA;
}