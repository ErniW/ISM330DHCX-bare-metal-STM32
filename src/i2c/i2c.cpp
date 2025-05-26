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
        if(!read(address, reg, &currentData, 1))
            continue;

        //update the value with data
        uint8_t updatedData = data | currentData;

        //try writing the updated data
        if(!tryWrite(address, reg, updatedData))
            continue;
        
        uint8_t verifyData = 0;
        if(!read(address, reg, &verifyData, 1))
            continue;

        if(verifyData == updatedData)
            return;

        //zrobic dzialanie
    }
    
    

    //if we went this far, restart the i2c by bit banging SDA 9 times
    //faultHandler();
}

bool I2C::waitForFlag(uint32_t reg, uint32_t flag){
    uint16_t timeout = I2C_TIMEOUT_VAL;

    while(reg & flag){
        if(checkErrors(--timeout))
            return false;
    };

    return true;
}

bool I2C::tryWrite(uint8_t address, uint8_t reg, uint8_t data){
    uint16_t timeout = I2C_TIMEOUT_VAL;

    while(_i2c->SR2 & I2C_SR2_BUSY){
        if(checkErrors(--timeout))
            return false;
    };

    // if(!waitForFlag(_i2c->SR2, I2C_SR2_BUSY)) return false;
    _i2c->CR1 |= I2C_CR1_START;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_SB)){
        if(checkErrors(--timeout))
            return false;
    };
    // if(!waitForFlag(_i2c->SR1, I2C_SR1_SB)) return false;
    _i2c->DR = address << 1;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_ADDR)){
        if(checkErrors(--timeout))
            return false;
    };
    // if(!waitForFlag(_i2c->SR1, I2C_SR1_ADDR)) return false;
    (void)_i2c->SR2;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_TXE)){
        if(checkErrors(--timeout))
            return false;
    };
    // if(!waitForFlag(_i2c->SR1, I2C_SR1_TXE)) return false;
    _i2c->DR = reg;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_TXE)){
        if(checkErrors(--timeout))
            return false;
    };
    // if(!waitForFlag(_i2c->SR1, I2C_SR1_TXE)) return false;
    _i2c->DR = data;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_BTF)){
        if(checkErrors(--timeout))
            return false;
    };
    // if(!waitForFlag(_i2c->SR1, I2C_SR1_BTF)) return false;
    _i2c->CR1 |= I2C_CR1_STOP;

    return true;
}

uint8_t error_count = 0;

uint8_t I2C::checkErrors(uint16_t timeout) {
   
    uint8_t state = I2C_OK;
    
    if(!timeout){
        // I2C1_manualRestart();
        // init();
        state = I2C_ERROR_TIMEOUT;
        // return state;
    }
    else if (_i2c->SR1 & I2C_SR1_BERR) {
        _i2c->SR1 &= ~I2C_SR1_BERR;
        state = I2C_ERROR_BERR; 
    }
    else if (_i2c->SR1 & I2C_SR1_ARLO) {
        _i2c->SR1 &= ~I2C_SR1_ARLO;
        state = I2C_ERROR_ARLO; 
    }
    else if (_i2c->SR1 & I2C_SR1_AF) {
        _i2c->SR1 &= ~I2C_SR1_AF;
        state = I2C_ERROR_AF;
    }
    else if (_i2c->SR1 & I2C_SR1_OVR) {
        _i2c->SR1 &= ~I2C_SR1_OVR;
        state = I2C_ERROR_OVR;
    }
    else if (_i2c->SR1 & I2C_SR1_TIMEOUT) {
        _i2c->SR1 &= ~I2C_SR1_TIMEOUT;
        state = I2C_ERROR_TIMEOUT;
    }

    // //to poprawić
    // if(state){
    //     _i2c->CR1 |= I2C_CR1_STOP;

    //     error_count++;
    //     if(error_count == I2C_RETRIES){
    //         I2C1_manualRestart();
    //         init();
    //     }
    //     return state;
    // }
    
    // //to zawsze czyści z odliczaniem, nie chcę tego w ten sposób
    // error_count = 0;
    return state;
}

bool I2C::read(uint8_t address, uint8_t reg, uint8_t* buffer, int n){
    uint16_t timeout = I2C_TIMEOUT_VAL;

    while(_i2c->SR2 & I2C_SR2_BUSY){
        if(checkErrors(--timeout))
        {
            _i2c->SR2 = 0;
            _i2c->SR1 = 0;
            return false;
        }
            
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

    _i2c->CR1 |= I2C_CR1_START;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_SB)){
        if(checkErrors(--timeout))
            return false;
    };

    _i2c->DR = address << 1 | 1;

    timeout = I2C_TIMEOUT_VAL;
    while(!(_i2c->SR1 & I2C_SR1_ADDR)){
        if(checkErrors(--timeout))
            return false;
    };

    (void)_i2c->SR2;

    _i2c->CR1 |= I2C_CR1_ACK;

    while (n > 0) {
        timeout = I2C_TIMEOUT_VAL;
        while (!(_i2c->SR1 & I2C_SR1_RXNE)) {
            if (checkErrors(--timeout))
                return false;
        }

        if (n == 1) {
            _i2c->CR1 &= ~I2C_CR1_ACK;
            _i2c->CR1 |= I2C_CR1_STOP;
        }

        *buffer++ = _i2c->DR;
        n--;
    }

    return true;
}