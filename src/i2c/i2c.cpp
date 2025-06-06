#include "i2c.h"

I2C::I2C (I2C_TypeDef* i2c) : _i2c(i2c) {};

void I2C::init(){
    _i2c->CR1 &=~ I2C_CR1_PE;
    _i2c->CR1 |= I2C_CR1_SWRST;
    _i2c->CR1 &=~ I2C_CR1_SWRST;
    while (_i2c->SR2 & I2C_SR2_BUSY);

    _i2c->CR2 |= (PCLK1 / 1000000);
    _i2c->CCR = PCLK1 / (3 * I2C_FREQ) | I2C_CCR_FS;
    _i2c->TRISE = (I2C_FAST_MODE_MAX_RISE_TIME * (PCLK1 / 1000000))/1000 + 1;
    
    _i2c->CR1 |= I2C_CR1_PE;


}

//dodać update mask
void I2C::write(uint8_t address, uint8_t reg, uint8_t data){

    uint8_t retries = I2C_WRITE_RETRIES;

    while(retries--){

        //read register value, store it to verify later
        uint8_t currentData = 0;
        if(!tryRead(address, reg, &currentData, 1))
            continue;

        //update the value with data
        uint8_t updatedData = data | currentData;

        //try writing the updated data
        if(!tryWrite(address, reg, updatedData))
            continue;
        
        uint8_t verifyData = 0;
        if(!tryRead(address, reg, &verifyData, 1))
            continue;

        if(verifyData == updatedData)
            return;

        if(retries == 1){
            I2C1_manualRestart();
            init();
        }
    }
    
    

    //if we went this far, restart the i2c by bit banging SDA 9 times
    //faultHandler();
}

bool I2C::waitForFlagSet(volatile uint32_t& reg, uint32_t flag){
    uint16_t timeout = I2C_TIMEOUT_VAL;

    while(!(reg & flag)){
        if(checkErrors(--timeout))
            return false;
    };

    return true;
}

bool I2C::waitForFlagClear(volatile uint32_t& reg, uint32_t flag){
    uint16_t timeout = I2C_TIMEOUT_VAL;

    while(reg & flag){
        if(checkErrors(--timeout))
            return false;
    };

    return true;
}

bool I2C::tryWrite(uint8_t address, uint8_t reg, uint8_t data){

    if(!waitForFlagClear(_i2c->SR2, I2C_SR2_BUSY)) return false;
    _i2c->CR1 |= I2C_CR1_START;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_SB)) return false;
    _i2c->DR = address << 1;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_ADDR)) return false;
    (void)_i2c->SR2;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_TXE)) return false;
    _i2c->DR = reg;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_TXE)) return false;
    _i2c->DR = data;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_BTF)) return false;
    _i2c->CR1 |= I2C_CR1_STOP;

    return true;
}

uint8_t I2C::checkErrors(uint16_t timeout) {
    
    if(!timeout){
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_TIMEOUT;
    }
    else if (_i2c->SR1 & I2C_SR1_BERR) {
        _i2c->SR1 &= ~I2C_SR1_BERR;
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_BERR; 
    }
    else if (_i2c->SR1 & I2C_SR1_ARLO) {
        _i2c->SR1 &= ~I2C_SR1_ARLO;
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_ARLO; 
    }
    else if (_i2c->SR1 & I2C_SR1_AF) {
        _i2c->SR1 &= ~I2C_SR1_AF;
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_AF;
    }
    else if (_i2c->SR1 & I2C_SR1_OVR) {
        _i2c->SR1 &= ~I2C_SR1_OVR;
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_OVR;
    }
    else if (_i2c->SR1 & I2C_SR1_TIMEOUT) {
        _i2c->SR1 &= ~I2C_SR1_TIMEOUT;
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_TIMEOUT;
    }

    return I2C_OK;
}

bool I2C::read(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t n){

    static uint8_t error_counter = 0;

    if(!tryRead(address, reg, buffer, n)){
        error_counter++;

        if(error_counter == I2C_READ_RETRIES){
            I2C1_manualRestart();
            init();
            error_counter = 0;
        }

        return false;
    }

    error_counter = 0;
    return true;
}

bool I2C::tryRead(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t n){

    if(!waitForFlagClear(_i2c->SR2, I2C_SR2_BUSY)) return false;
    _i2c->CR1 |= I2C_CR1_START;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_SB)) return false;
    _i2c->DR = address << 1;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_ADDR)) return false;
    (void)_i2c->SR2;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_TXE)) return false;
    _i2c->DR = reg;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_TXE)) return false;
    _i2c->CR1 |= I2C_CR1_START;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_SB)) return false;
    _i2c->DR = address << 1 | 1;

    if(!waitForFlagSet(_i2c->SR1, I2C_SR1_ADDR)) return false;
    (void)_i2c->SR2;

    _i2c->CR1 |= I2C_CR1_ACK;

    while (n > 0) {
        if(!waitForFlagSet(_i2c->SR1, I2C_SR1_RXNE)) return false;

        if (n == 1) {
            _i2c->CR1 &= ~I2C_CR1_ACK;
            _i2c->CR1 |= I2C_CR1_STOP;
        }

        *buffer++ = _i2c->DR;
        n--;
    }

    return true;
}