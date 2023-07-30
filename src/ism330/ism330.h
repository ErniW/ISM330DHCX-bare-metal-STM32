#include "../i2c/i2c.h"
#include "stm32f446xx.h"

class ISM330DHCX : protected I2C {
public:
    ISM330DHCX();
    void readGyro();
    void readAccel();
    void getIMU();

    void enablePedometer();
    void enableSingleTap();
    void enableDoubleTap();
    void enableTiltDetection();
    void enableFreeFall();


private:
    uint8_t _address();
    // virtual void i2c_write();
    // virtual void i2c_read();
    // virtual void i2c_readMany();
};