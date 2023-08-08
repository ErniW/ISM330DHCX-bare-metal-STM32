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

I2C* i2c = new I2C(I2C1);
ISM330DHCX* ISM330 = new ISM330DHCX((uint8_t)ADDRESS, i2c);

int main(){

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER |= PB8_AF_MODE | PB9_AF_MODE;
    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;
    GPIOB->PUPDR |= PB8_PULLUP | PB9_PULLUP;
    GPIOB->AFR[1] |= PB8_AF4_I2C_SCL | PB9_AF4_I2C_SDA;

    tx_init();
    SysTick_Init();

    i2c->init();
    ISM330->init(
        FREQ_416_HZ,
        ACCEL_2G,
        FREQ_416_HZ,
        GYRO_2000_DPS
    );

    // ISM330->enablePedometer();

    ISM330->enableSingleTap();

    while(1){

        volatile uint8_t tap;

        ISM330->readSingleTap(tap);

        if(tap & SINGLE_TAP){
             printf("Tap\t %u", tap);

            if(tap & isTapX){
                if(tap & tapSign) printf("+X tap\n");
                else printf("-X tap \n");
            }
            else if(tap & isTapY){
                if(tap & tapSign) printf("+Y tap \n");
                else printf("-Y tap \n");
            }
            else if(tap & isTapZ){
                if(tap & tapSign) printf("+Z tap \n");
                else printf("-Z tap \n");
            }

        }
        delay_ms(10);

        // int16_t ax, ay, az, gx, gy, gz;

        // readAccel(ax, ay, az);
        // readGyro(gx, gy, gz);

        // printf("Acc: %d, %d, %d \tGyro: %d, %d, %d\n", ax, ay, az, gx, gy, gz);

        // delay_ms(50);

        // float x, y, z;
        // ISM330->getIMU(x,y,z);
        // printf("Roll: %.2f\t Pitch: %.2f\t Yaw: %.2f\n", x, y, z);
        // // delay_ms(10);

        // int16_t steps;

        // ISM330->readPedometer(steps);

        // printf("Steps: %d\n", steps);

        // delay_ms(100);
    }

}