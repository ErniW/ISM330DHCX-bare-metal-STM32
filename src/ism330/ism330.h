#pragma once

#include "i2c.h"
#include "stm32f446xx.h"
#include <memory>
#include <cstdio>
#include <math.h>

#define ADDRESS     0x6A

#define FUNC_CFG_ACCESS 0x01
#define EMB_FUNC_EN_A   0x04

#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL3_C     0x12

#define TAP_SRC     0x1C

#define READ_GYRO   0x22
#define READ_ACCEL  0x28

#define TAP_CFG0    0x56
#define TAP_CFG1    0x57
#define TAP_CFG2    0x58
#define TAP_THS_6D  0x59
#define INT_DUR2    0x5A
#define MD1_CFG     0x5E

#define EMB_FUNC_STEP_COUNTER_L 0x62

#define SW_RESET    0x01
#define AUTO_INC    (1 << 3)

#define FUNC_CFG_ACCESS_EN  (1 << 7)
#define PEDO_EN             (1 << 3)

#define INT_CLR_ON_READ (1 << 7)
#define TAP_X_EN        (1 << 3)
#define TAP_Y_EN        (1 << 2)
#define TAP_Z_EN        (1 << 1)
#define LATCHED_INT     (1 << 0)

#define TAP_THRESHOLD_X 9
#define TAP_THRESHOLD_Y 9
#define TAP_THRESHOLD_Z 9
#define TAP_SHOCK       2
#define TAP_QUIET       (1 << 2)

#define INTERRUPTS_EN   (1 << 7)
#define INT1_SINGLE_TAP (1 << 6)
#define SINGLE_TAP      (1 << 6)

#define TAP_X            (1 << 2)
#define TAP_Y            (1 << 1)
#define TAP_Z            (1 << 0)
#define TAP_SIGN         (1 << 3)

enum TapEvent{
    TAP_NO_EVENT = 0,
    TAP_EVENT_X_POSITIVE = TAP_SIGN | TAP_X,
    TAP_EVENT_X_NEGATIVE = TAP_X,
    TAP_EVENT_Y_POSITIVE = TAP_SIGN | TAP_Y,
    TAP_EVENT_Y_NEGATIVE = TAP_Y,
    TAP_EVENT_Z_POSITIVE = TAP_SIGN | TAP_Z,
    TAP_EVENT_Z_NEGATIVE = TAP_Z
};

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

    void enablePedometer();
    uint16_t readPedometer();

    void enableSingleTap();
    uint8_t readSingleTap();

private:
    uint8_t _address;
    I2C* _i2c;
};