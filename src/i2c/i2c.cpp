#include "i2c.h"

//#define I2C_100KHZ 80
#define SD_MODE_MAX_RISE_TIME 17

I2C::I2C (I2C_TypeDef* i2c) : _i2c(i2c) {};

void I2C::init(){
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &=~ I2C_CR1_SWRST;

    I2C1->CR2 |= 45; //16;
    I2C1->CCR = 225; //I2C_100KHZ;
    I2C1->TRISE = SD_MODE_MAX_RISE_TIME;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C::write(uint8_t address, uint8_t reg, uint8_t data){

    volatile int tmp;

    while(_i2c->SR2 & I2C_SR2_BUSY);
    _i2c->CR1 |= I2C_CR1_START;

    while(!(_i2c->SR1 & I2C_SR1_SB));
    _i2c->DR = address << 1;

    while(!(_i2c->SR1 & I2C_SR1_ADDR));
    tmp = _i2c->SR2;

    while(!(_i2c->SR1 & I2C_SR1_TXE));
    _i2c->DR = reg;

    while(!(_i2c->SR1 & I2C_SR1_TXE));
    _i2c->DR = data;

    while(!(_i2c->SR1 & I2C_SR1_BTF));
    _i2c->CR1 |= I2C_CR1_STOP;
}

void I2C::read(uint8_t address, uint8_t reg, uint8_t* buffer, int n){
    
    volatile int tmp;

    while(_i2c->SR2 & I2C_SR2_BUSY);
    _i2c->CR1 |= I2C_CR1_START;

    while(!(_i2c->SR1 & I2C_SR1_SB));
    _i2c->DR = address << 1;

    while(!(_i2c->SR1 & I2C_SR1_ADDR));
    tmp = _i2c->SR2;

    while(!(_i2c->SR1 & I2C_SR1_TXE));
    _i2c->DR = reg;

    while(!(_i2c->SR1 & I2C_SR1_TXE));
    _i2c->CR1 |= I2C_CR1_START;

    while(!(_i2c->SR1 & I2C_SR1_SB));
    _i2c->DR = address << 1 | 1;

    while(!(_i2c->SR1 & I2C_SR1_ADDR));
    tmp = _i2c->SR2;

    _i2c->CR1 |= I2C_CR1_ACK;

    while(n >0){
        if(n == 1){
            _i2c->CR1 &=~ I2C_CR1_ACK;
            _i2c->CR1 |= I2C_CR1_STOP;

            while (!(_i2c->SR1 & I2C_SR1_RXNE));
            *buffer++ = _i2c->DR;

            break;
        }
        else{

            while (!(_i2c->SR1 & I2C_SR1_RXNE));
            *buffer++ = _i2c->DR;
            n--;
        }
    }    
}