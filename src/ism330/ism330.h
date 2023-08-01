#pragma once


#include "../i2c/i2c.h"
#include "stm32f446xx.h"
#include <memory>

#define ADDRESS     0x6A

class ISM330DHCX {
public:
    ISM330DHCX(uint8_t address, std::unique_ptr<I2C>& i2c);
    void readGyro();
    void readAccel();
    void getIMU();

    // void enablePedometer();
    // void enableSingleTap();
    // void enableDoubleTap();
    // void enableTiltDetection();
    // void enableFreeFall();

    // void interruptHandler();
private:
    uint8_t _address;
    // virtual void i2c_write();
    // virtual void i2c_read();
    // virtual void i2c_readMany();
};