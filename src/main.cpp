#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include "i2c.h"
#include "ism330.h"
#include "uart.h"
#include "systick.h"

#include <memory>
#include <cstdio>
#include <string.h>

#define PB8_AF_MODE (1 << 17)
#define PB9_AF_MODE (1 << 19)

#define PB8_AF4_I2C_SCL (1 << 2)
#define PB9_AF4_I2C_SDA (1 << 6)

#define PB8_PULLUP (1 << 16)
#define PB9_PULLUP (1 << 18)

I2C i2c(I2C1);
ISM330DHCX ISM330((uint8_t)ADDRESS, &i2c);

int main(){

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER |= PB8_AF_MODE | PB9_AF_MODE;
    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;
    GPIOB->PUPDR |= PB8_PULLUP | PB9_PULLUP;
    GPIOB->AFR[1] |= PB8_AF4_I2C_SCL | PB9_AF4_I2C_SDA;

    tx_init();
    SysTick_Init();

    i2c.init();
    ISM330.init(
        FREQ_416_HZ,
        ACCEL_2G,
        FREQ_104_HZ,
        GYRO_2000_DPS
    );

    uint16_t steps_counter = 0;

    // ISM330.enablePedometer();
    // ISM330.enableSingleTap();

    while(1){
        /*
            IMU ALGORITHM

            A basic IMU implementation with complementary filter.

            TODO: fix yaw frequency in gyroscope. Probably include interrupts on gyroscope data. Otherwise it will accumulate error with each second.
        */

        float roll, pitch, yaw;
        
        ISM330.getIMU(roll, pitch, yaw);
        printf("Roll: %.2f\t Pitch: %.2f\t Yaw: %.2f\n", roll, pitch, yaw);
        delay_ms(10);

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