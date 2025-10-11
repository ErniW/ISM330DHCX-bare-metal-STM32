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

bool errori2c = false;

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
    ISM330.init(
        FREQ_416_HZ,
        ACCEL_2G,
        FREQ_416_HZ,
        GYRO_1000_DPS
    );

    ISM330.gyroInterruptEnable();

    uint8_t cnt = 0;

    while(1){

        if(i2c._state == I2C_STATE_DATA_READY){
            switch(ISM330.state){
                case IMU_STATE_WAIT_FOR_GYRO_DATA:
                    ISM330.gyroDataReadyHandler();
                    ISM330.state = IMU_STATE_GET_ACCEL_DATA;
                    break;
                case IMU_STATE_WAIT_FOR_ACCEL_DATA:
                    ISM330.accelDataReadyHandler();
                    ISM330.state = IMU_STATE_COMPUTE_FUSION;
                    break;
            }

            i2c._state = I2C_STATE_IDLE;
        }
        else if(i2c._state == I2C_STATE_ERROR){
            I2C1_manualRestart();
            i2c.init();
            i2c._state = I2C_STATE_IDLE;
        }

        switch(ISM330.state){
            case IMU_STATE_WAIT_FOR_INTERRUPT:
                if(ISM330.isGyroDataReady)
                    ISM330.gyroInterruptHandler();
                break;
            case IMU_STATE_GET_GYRO_DATA:
                ISM330.gyroRequest();
                break;
            case IMU_STATE_GET_ACCEL_DATA:
                ISM330.accelRequest();
                break;
            case IMU_STATE_COMPUTE_FUSION:
                ISM330.computeIMU();

                if(cnt % 10 == 0)
                    printf("Quaternion: %.2f, %.2f, %.2f, %.2f\n", ISM330.quaternion.element.w, ISM330.quaternion.element.x, ISM330.quaternion.element.y, ISM330.quaternion.element.z);
                cnt++;
                
                ISM330.state = IMU_STATE_WAIT_FOR_INTERRUPT;
                break;
            default:
                break;
        }
    }

}

extern "C" void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & INT1_PIN){
        ISM330.isGyroDataReady = true;
        EXTI->PR |= INT1_PIN;
    }
}

extern "C" void I2C1_EV_IRQHandler(void) {
    i2c.IRQhandler();
}

extern "C" void I2C1_ER_IRQHandler(void){
    i2c.IRQerrorHandler();
    ISM330.state = IMU_STATE_WAIT_FOR_INTERRUPT;
}