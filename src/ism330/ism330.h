#pragma once


#include "../i2c/i2c.h"
#include "stm32f446xx.h"
#include <memory>

#define ADDRESS     0x6A

#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL3_C     0x12

#define READ_GYRO   0x22
#define READ_ACCEL  0x28

#define SW_RESET 0x01
#define AUTO_INC (1 << 3)
#define ACCELEROMETER_416HZ_2G (6 << 4)
#define GYROSCOPE_416HZ_2000DPS 0x6C

class ISM330DHCX {
public:
    ISM330DHCX(uint8_t address, I2C* i2c);
    void init();
    void readGyro(int16_t& x, int16_t& y, int16_t& z);
    void readAccel(int16_t& x, int16_t& y, int16_t& z);
    void getIMU();

    // void enablePedometer();
    // void enableSingleTap();
    // void enableDoubleTap();
    // void enableTiltDetection();
    // void enableFreeFall();

    // void interruptHandler();
private:
    uint8_t _address;
    I2C* _i2c;
    // virtual void i2c_write();
    // virtual void i2c_read();
    // virtual void i2c_readMany();
};