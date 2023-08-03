#pragma once

#include "stm32f446xx.h"

class I2C {
public:
    I2C(I2C_TypeDef* i2c);
    void init();
    void write(uint8_t address, uint8_t reg, uint8_t data);
    void read(uint8_t address, uint8_t reg, uint8_t* buffer, int n);
private:
    I2C_TypeDef* _i2c;
    uint8_t _state;
};