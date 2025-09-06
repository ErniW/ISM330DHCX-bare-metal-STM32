#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include "i2c.h"
#include "ism330.h"
#include "uart.h"
#include "systick.h"
#include "timer.h"

#include <memory>
#include <cstdio>
#include <string.h>

#define PA5_OUTPUT  (1 << 10)
#define LED_PIN     (1 << 5)

#define EXTI_C12    (2 << 0)
#define INT1_PIN    (1 << 12)

I2C i2c(I2C1);
ISM330DHCX ISM330((uint8_t)ADDRESS, &i2c);

extern "C" void __disable_irq(void);
extern "C" void __enable_irq(void);
extern "C" void __enable_fault_irq(void);

int main(){
    SCB->CPACR |= (0xF << 20);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1_gpioConfig();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= PA5_OUTPUT;

    __disable_irq();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[3] |= EXTI_C12;
    EXTI->IMR |= INT1_PIN;
    EXTI->RTSR |= INT1_PIN;

    NVIC_EnableIRQ(EXTI15_10_IRQn);
    __enable_irq();

    tx_init();
    SysTick_Init();

    i2c.init();

    while(1){
        // uint8_t whoami = 0;
        // i2c.read(0x6A, 0x0F, &whoami, 1); 
        // printf("Val: %x\n", whoami);

        i2c.write(0x6A, CTRL1_XL, 0xF);

        delay_ms(100);
    }

}

extern "C" void EXTI15_10_IRQHandler(void);

void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & INT1_PIN){
        ISM330.isGyroDataReady = true;
        EXTI->PR |= INT1_PIN;
    }
}

extern "C" void I2C1_EV_IRQHandler(void) {
    i2c.IRQhandler();
}