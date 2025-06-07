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

volatile bool isGyroDataReady = false;

int main(){

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

    

    uint16_t steps_counter = 0;

    // ISM330.enablePedometer();
    // ISM330.enableSingleTap();

    uint32_t previous_time = 0;

    float roll =0;
    float pitch = 0;
    float yaw = 0;

    float avgX = 0; 
    float avgY = 0;
    float avgZ = 0;

    delay_ms(100);
    for(uint8_t i=0; i<100; i++){
        int16_t gx,gy,gz = 0;
        ISM330.readGyro(gx,gy,gz);
        avgX += (float)gx;
        avgY += (float)gy;
        avgZ += (float)gz;
        delay_ms(10);
    }

    ISM330.gyroCalibrationX = avgX / 100.0;
    ISM330.gyroCalibrationY = avgY / 100.0;
    ISM330.gyroCalibrationZ = avgZ / 100.0;

    ISM330.gyroInterruptEnable();


    uint8_t cnt = 0;

    while(1){

        if(isGyroDataReady){
            ISM330.getIMU(roll,pitch,yaw);
            // printf("%.2f, %.2f, %.2f\n", roll,pitch,yaw);

            if(cnt % 20 == 0)
                printf("Quaternion: %.2f, %.2f, %.2f, %.2f\n", ISM330.quaternion.w, ISM330.quaternion.x, ISM330.quaternion.y, ISM330.quaternion.z);

            cnt++;
            
            isGyroDataReady = false;
            GPIOA->ODR ^= LED_PIN;
        }

        /*
            TAP EVENT DETECTION

            Please be aware that further calibration of threshold is required
        */

        // uint8_t tap_event = ISM330.readSingleTap();

        // switch(tap_event){
        //     case TAP_EVENT_X_POSITIVE:
        //         printf("+X tap\n");
        //         break;
        //     case TAP_EVENT_X_NEGATIVE:
        //         printf("-X tap\n");
        //         break;
        //     case TAP_EVENT_Y_POSITIVE:
        //         printf("+Y tap\n");
        //         break;
        //     case TAP_EVENT_Y_NEGATIVE:
        //         printf("-Y tap\n");
        //         break;
        //     case TAP_EVENT_Z_POSITIVE:
        //         printf("+Z tap\n");
        //         break;
        //     case TAP_EVENT_Z_NEGATIVE:
        //         printf("-Z tap\n");
        //         break;
        // }

        // delay_ms(10);


        /*
            PEDOMETER

            Please be aware that pedometer starts transmitting after few counted steps.
        */

        // uint16_t current_steps = ISM330.readPedometer();
        
        // if(current_steps != steps_counter){
        //     steps_counter = current_steps;
        //     printf("Steps: %d\n", steps_counter);
        // }

        // delay_ms(100);
    }

}

extern "C" void EXTI15_10_IRQHandler(void);

void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & INT1_PIN){
        isGyroDataReady = true;
        EXTI->PR |= INT1_PIN;
    }
}