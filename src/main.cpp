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
    ISM330.init(
        FREQ_416_HZ,
        ACCEL_2G,
        FREQ_416_HZ,
        GYRO_1000_DPS
    );

    ISM330.gyroInterruptEnable();

    while(1){
        if(ISM330.isGyroDataReady && ISM330.state == IMU_STATE_IDLE){
            ISM330.state = IMU_STATE_GET_GYRO_DATA;
            ISM330.isGyroDataReady = false;
        }

        switch(ISM330.state){
            case IMU_STATE_IDLE:

                break;
            case IMU_STATE_GET_GYRO_DATA:
                ISM330.requestGyro();
                break;
            case IMU_STATE_GET_ACCEL_DATA:
                ISM330.requestAccel();
                break;
            case IMU_STATE_COMPUTE_FUSION:
                //ISM330.getIMU();
                // printf("%d, %d, %d\n", ISM330.ax, ISM330.ay, ISM330.az);
                ISM330.state = IMU_STATE_IDLE;
                break;
        }

        //tutaj pobierać bezpośrednio z obiektu i sprawdzać adres z którego jest data ready.
        if(i2c._state == I2C_STATE_DATA_READY){
            switch(ISM330.state){
                case IMU_STATE_WAIT_FOR_GYRO_DATA:
                    ISM330.gx = (i2c._packet.buffer[1] << 8 | i2c._packet.buffer[0]);
                    ISM330.gy = (i2c._packet.buffer[3] << 8 | i2c._packet.buffer[2]);
                    ISM330.gz = (i2c._packet.buffer[5] << 8 | i2c._packet.buffer[4]);
                    ISM330.state = IMU_STATE_GET_ACCEL_DATA;
                    break;
                case IMU_STATE_WAIT_FOR_ACCEL_DATA:
                    ISM330.ax = (i2c._packet.buffer[1] << 8 | i2c._packet.buffer[0]);
                    ISM330.ay = (i2c._packet.buffer[3] << 8 | i2c._packet.buffer[2]);
                    ISM330.az = (i2c._packet.buffer[5] << 8 | i2c._packet.buffer[4]);
                    ISM330.state = IMU_STATE_COMPUTE_FUSION;
                    break;
            }

            printf("%d, %d, %d - %d, %d, %d\n", ISM330.gx, ISM330.gy, ISM330.gz,  ISM330.ax,  ISM330.ay, ISM330.az);

            i2c._state =I2C_STATE_IDLE;
        }
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