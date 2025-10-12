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

I2C i2c(I2C1, DMA1, DMA1_Stream0);
ISM330DHCX ISM330((uint8_t)ISM330_ADDRESS, &i2c);

extern "C" void __disable_irq(void);
extern "C" void __enable_irq(void);
extern "C" void __enable_fault_irq(void);

int main(){
    SCB->CPACR |= (0xF << 20);

    I2C1_gpioConfig();
    tx_init();
    SysTick_Init();

    i2c.init();
    ISM330.init(
        FREQ_416_HZ,
        ACCEL_2G,
        FREQ_416_HZ,
        GYRO_1000_DPS
    );

    __disable_irq();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[3] |= EXTI_C12;
    EXTI->IMR |= INT1_PIN;
    EXTI->RTSR |= INT1_PIN;
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    __enable_irq();

    ISM330.gyroCalibrate(100);
    ISM330.gyroInterruptEnable();
    
    uint8_t cnt = 0;

    while(1){

        /*
            PROCESS I2C DMA TRANSFER COMPLETE 
            ------------------------------------
            Whenever DMA transfer is complete, set I2C state
            to I2C_STATE_DATA_READY. Then process the state of device
            which requested the data. Bus is released immediately after.

            I2C DMA reading allows to do other things in between of readings.
        */

        volatile uint8_t i2cState = i2c.getState();

        if(i2cState == I2C_STATE_DATA_READY){
            /*
                OWNERSHIP OF I2C BUS
                -------------------------------
                Check which device awaits the data to process its
                state machine.

                It's obsolete for a single device but for multiple
                devices it allows to distinguish them.
            */
            if(i2c.checkOwnership() == ISM330_ADDRESS){
                /*
                    UPDATE ASYNCHRONOUS STATE MACHINE
                    ------------------------------------------
                    Acquire data requested by device state machine
                    and process to its next step.
                */
                switch(ISM330.getState()){
                    case IMU_STATE_WAIT_FOR_GYRO_DATA:
                        ISM330.gyroDataReadyHandler();
                        ISM330.setState(IMU_STATE_GET_ACCEL_DATA);
                        break;
                    case IMU_STATE_WAIT_FOR_ACCEL_DATA:
                        ISM330.accelDataReadyHandler();
                        ISM330.setState(IMU_STATE_COMPUTE_FUSION);
                        break;
                }
            }

            i2c.setState(I2C_STATE_IDLE);
        }
        else if(i2cState == I2C_STATE_ERROR){
            I2C1_manualRestart();
            i2c.init();
            i2c.setState(I2C_STATE_IDLE);
        }

        /*
            PROCESS ISM330 FUSION STATE MACHINE
            ------------------------------------
            Sequence:
            1. Await gyroscope interrupt (each reading time is depended on gyro
               data because it's mandatory for proper integration).
            2. Request Gyro data and wait for DMA transfer complete.
            3. Request Accel data and wait for DMA transfer complete.
            4. Compute fusion and print it for adafruit 3d model viewer.
            5. Wait for another interrupt.

            Steps that wait for asynchronous data is handled by 
            I2C_DATA_READY_STATE.

            https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/
        */
        switch(ISM330.getState()){
            case IMU_STATE_WAIT_FOR_INTERRUPT:
                if(ISM330.gyroIsAvailable())
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
                
                ISM330.setState(IMU_STATE_WAIT_FOR_INTERRUPT);
                break;
            default:
                break;
        }
    }
}

extern "C" void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & INT1_PIN){
        ISM330.gyroSetDataAvailable();
        EXTI->PR |= INT1_PIN;
    }
}

extern "C" void I2C1_ER_IRQHandler(void){
    i2c.IRQerrorHandler();
    ISM330.setState(IMU_STATE_WAIT_FOR_INTERRUPT);
}

extern "C" void DMA1_Stream0_IRQHandler(void){
    i2c.IRQdmaEventHandler();
}