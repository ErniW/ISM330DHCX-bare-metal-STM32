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

    uint8_t retries = I2C_RETRIES;


    while(retries--){

        //read register value, store it to verify later
        uint8_t currentData = 0;
        read(address, reg, &currentData, 1);

        //update the value with data
        uint8_t updatedData = data | currentData;

        //try writing the updated data
        if(tryWrite(address, reg, updatedData))
        {
            uint8_t verifyData = 0;
            read(address, reg, &verifyData, 1);

            if(verifyData == updatedData)
                return;
        }
    }
    
    //if we went this far, restart the i2c by bit banging SDA 9 times
    //faultHandler();
}

bool I2C::tryWrite(uint8_t address, uint8_t reg, uint8_t data){
    uint16_t timeout = I2C_TIMEOUT_VAL;

    while(_i2c->SR2 & I2C_SR2_BUSY){
        if(checkErrors(--timeout))
            return false;
    };
    _i2c->CR1 |= I2C_CR1_START;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_SB)){
        if(checkErrors(--timeout))
            return false;
    };
    _i2c->DR = address << 1;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_ADDR)){
        if(checkErrors(--timeout))
            return false;
    };
    (void)_i2c->SR2;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_TXE)){
        if(checkErrors(--timeout))
            return false;
    };
    _i2c->DR = reg;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_TXE)){
        if(checkErrors(--timeout))
            return false;
    };
    _i2c->DR = data;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_BTF)){
        if(checkErrors(--timeout))
            return false;
    };
    _i2c->CR1 |= I2C_CR1_STOP;

    return true;
}

uint8_t I2C::checkErrors(uint16_t timeout) {

    if(!timeout){
        return I2C_ERROR_TIMEOUT;
    }
    else if (_i2c->SR1 & I2C_SR1_BERR) {
        _i2c->SR1 &= ~I2C_SR1_BERR;
        return I2C_ERROR_BERR; 
    }
    else if (_i2c->SR1 & I2C_SR1_ARLO) {
        _i2c->SR1 &= ~I2C_SR1_ARLO;
        return I2C_ERROR_ARLO; 
    }
    else if (_i2c->SR1 & I2C_SR1_AF) {
        _i2c->SR1 &= ~I2C_SR1_AF;
        return I2C_ERROR_AF;
    }
    else if (_i2c->SR1 & I2C_SR1_OVR) {
        _i2c->SR1 &= ~I2C_SR1_OVR;
        return I2C_ERROR_OVR;
    }
    else if (_i2c->SR1 & I2C_SR1_TIMEOUT) {
        _i2c->SR1 &= ~I2C_SR1_TIMEOUT;
        return I2C_ERROR_TIMEOUT;
    }

    return I2C_OK;
}

bool I2C::read(uint8_t address, uint8_t reg, uint8_t* buffer, int n){
    
    volatile int tmp;

    uint16_t timeout = I2C_TIMEOUT_VAL;

    while(_i2c->SR2 & I2C_SR2_BUSY){
        if(checkErrors(--timeout))
            return false;
    };

    _i2c->CR1 |= I2C_CR1_START;

    // while(!(_i2c->SR1 & I2C_SR1_SB));

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_SB)){
        if(checkErrors(--timeout))
            return false;
    };

    _i2c->DR = address << 1;

    // while(!(_i2c->SR1 & I2C_SR1_ADDR));

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_ADDR)){
        if(checkErrors(--timeout))
            return false;
    };

    tmp = _i2c->SR2;

    // while(!(_i2c->SR1 & I2C_SR1_TXE));

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_TXE)){
        if(checkErrors(--timeout))
            return false;
    };

    _i2c->DR = reg;

    // while(!(_i2c->SR1 & I2C_SR1_TXE));

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_TXE)){
        if(checkErrors(--timeout))
            return false;
    };

    _i2c->CR1 |= I2C_CR1_START;

    // while(!(_i2c->SR1 & I2C_SR1_SB));
     timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_SB)){
        if(checkErrors(--timeout))
            return false;
    };

    _i2c->DR = address << 1 | 1;

    // while(!(_i2c->SR1 & I2C_SR1_ADDR));

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_ADDR)){
        if(checkErrors(--timeout))
            return false;
    };

    tmp = _i2c->SR2;

    _i2c->CR1 |= I2C_CR1_ACK;

    while(n >0){
        if(n == 1){
            _i2c->CR1 &=~ I2C_CR1_ACK;
            _i2c->CR1 |= I2C_CR1_STOP;

            // while (!(_i2c->SR1 & I2C_SR1_RXNE));

            timeout = I2C_TIMEOUT_VAL;
            while(!(_i2c->SR1 & I2C_SR1_RXNE)){
                if(checkErrors(--timeout))
                    return false;
            };

            *buffer++ = _i2c->DR;

            break;
        }
        else{

            // while (!(_i2c->SR1 & I2C_SR1_RXNE));

            timeout = I2C_TIMEOUT_VAL;
            while(!(_i2c->SR1 & I2C_SR1_RXNE)){
                if(checkErrors(--timeout))
                    return false;
            };

            *buffer++ = _i2c->DR;
            n--;
        }
    }    

    return true;
}