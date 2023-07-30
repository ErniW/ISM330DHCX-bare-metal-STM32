#include "stm32f446xx.h"

class I2C {
public:
    I2C();
protected:
    void read();
    void readBuffer();
    void write();
private:
    I2C_TypeDef _i2c;
    uint8_t _state;
};