#pragma once

#include "stm32f446xx.h"

#define SYS_CLK 16000000
#define PCLK1   SYS_CLK
#define I2C_FREQ 400000
#define I2C_FAST_MODE_MAX_RISE_TIME 300

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