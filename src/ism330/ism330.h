#pragma once

#include "i2c.h"
#include "stm32f446xx.h"
#include <memory>
#include <cstdio>
#include <math.h>

#define ADDRESS     0x6A

#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL3_C     0x12

#define READ_GYRO   0x22
#define READ_ACCEL  0x28

#define SW_RESET 0x01
#define AUTO_INC (1 << 3)

enum freq{
    POWER_DOWN,
    FREQ_12_5_HZ,
    FREQ_26_HZ,
    FREQ_52_HZ,
    FREQ_104_HZ,
    FREQ_208_HZ,
    FREQ_416_HZ,
    FREQ_833_HZ,
    FREQ_1_66_KHZ,
    FREQ_3_33_KHZ,
    FREQ_6_66_KHZ,
    ACCEL_FREQ_1_6_HZ
};

enum accelSensitivity{
    ACCEL_2G,
    ACCEL_16G,
    ACCEL_4G,
    ACCEL_8G
};

enum gyroDPS{
    GYRO_4000_DPS = 1,
    GYRO_125_DPS = 2,
    GYRO_250_DPS = 0,
    GYRO_500_DPS = 4,
    GYRO_1000_DPS = 8,
    GYRO_2000_DPS = 12,
};

class ISM330DHCX {
public:
    ISM330DHCX(uint8_t address, I2C* i2c);
    void init(uint8_t accelFreq, uint8_t accelSensitivity, uint8_t gyroFreq, uint8_t gyroDPS);
    void readGyro(int16_t& x, int16_t& y, int16_t& z);
    void readAccel(int16_t& x, int16_t& y, int16_t& z);
    void getIMU(float& roll, float& pitch, float& yaw);

    // void enablePedometer();
    // void enableSingleTap();
    // void enableDoubleTap();
    // void enableTiltDetection();
    // void enableFreeFall();

    // void interruptHandler();
private:
    uint8_t _address;
    I2C* _i2c;
};