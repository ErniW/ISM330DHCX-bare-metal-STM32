#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include "i2c.h"
#include "ism330/ism330.h"
#include "uart.h"

#include <memory>
#include <cstdio>
#include <string.h>

extern "C" {
    int __io_putchar(int ch);
}

int __io_putchar(int ch){
    tx_send(ch);
    return ch;
}

#define PA5_OUTPUT  (1 << 10)
#define LED_PIN     (1 << 5)

#define PB8_AF_MODE (1 << 17)
#define PB9_AF_MODE (1 << 19)

#define PB8_AF4_I2C_SCL (1 << 2)
#define PB9_AF4_I2C_SDA (1 << 6)

#define PB8_PULLUP (1 << 16)
#define PB9_PULLUP (1 << 18)

#define MPU6050_ADDRESS 0x68
#define RESET           0x6B
#define ACCEL_CONFIG    0x1C
#define GYRO_CONFIG     0x1B
#define READ_ACCEL      0x3B
#define READ_GYRO       0x43

std::unique_ptr<I2C> i2c(new I2C(I2C1));
//std::unique_ptr<ISM330DHCX> ISM330(new ISM330DHCX((uint8_t)ADDRESS, i2c));

int main(){


    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER |= PB8_AF_MODE | PB9_AF_MODE;
    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;
    GPIOB->PUPDR |= PB8_PULLUP | PB9_PULLUP;
    GPIOB->AFR[1] |= PB8_AF4_I2C_SCL | PB9_AF4_I2C_SDA;

    i2c->init();



    i2c->write(MPU6050_ADDRESS, RESET, 0x00);
    i2c->write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00);
    i2c->write(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);


    tx_init();


    while(1){


        char accelData[6];
        char gyroData[6];

        i2c->read(MPU6050_ADDRESS, READ_ACCEL, (char*)accelData, 6);

        int16_t ax = (accelData[0]<<8 | accelData[1]);
        int16_t ay = (accelData[2]<<8 | accelData[3]);
        int16_t az = (accelData[4]<<8 | accelData[5]);

        i2c->read(MPU6050_ADDRESS, READ_GYRO, (char*)gyroData, 6);

        int16_t gx = (gyroData[0]<<8 | gyroData[1]);
        int16_t gy = (gyroData[2]<<8 | gyroData[3]);
        int16_t gz = (gyroData[4]<<8 | gyroData[5]);

        printf("Acc: %d, %d, %d \tGyro: %d, %d, %d\n", ax, ay, az, gx, gy, gz);

        
      //  printf("Test\n");
    }

}







        // delay_ms(50);

        // printf("test\n");