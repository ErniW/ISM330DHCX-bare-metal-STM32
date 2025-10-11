#include "i2c.h"
#include <string.h>
#include <cstdio>

I2C::I2C (I2C_TypeDef* i2c) : _i2c(i2c) {};

void I2C::init(){
    _i2c->CR1 &=~ I2C_CR1_PE;
    _i2c->CR1 |= I2C_CR1_SWRST;
    _i2c->CR1 &=~ I2C_CR1_SWRST;
     if(!waitForFlagClear(_i2c->SR2, I2C_SR2_BUSY)) NVIC_SystemReset();
    _i2c->CR2 |= (PCLK1 / 1000000);
    _i2c->CCR = PCLK1 / (3 * I2C_FREQ) | I2C_CCR_FS;
    _i2c->TRISE = (I2C_FAST_MODE_MAX_RISE_TIME * (PCLK1 / 1000000))/1000 + 1;
    
    _i2c->CR1 |= I2C_CR1_PE;
    _packet = {0};
    _state = I2C_STATE_IDLE;

    if (_i2c == I2C1) {
        NVIC_EnableIRQ(I2C1_EV_IRQn);
        NVIC_EnableIRQ(I2C1_ER_IRQn);
    } else if (_i2c == I2C2) {
        NVIC_EnableIRQ(I2C2_EV_IRQn);
        NVIC_EnableIRQ(I2C2_ER_IRQn);
    } else if (_i2c == I2C3) {
        NVIC_EnableIRQ(I2C3_EV_IRQn);
        NVIC_EnableIRQ(I2C3_ER_IRQn);
    }
}

/*
    I2C DATA WRITE PROCEDURE
    -------------------------------------
    1. Read the target register
    2. Locally update the data
    3. Update the register
    4. Read again to verify.

    There are I2C_WRITE_RETRIES retries, before last attempt
    the I2C bus is restarted and then we have a final attempt.

    If you whish you can include faultHandler.

    Further alternative would be storing every register locally
    to avoid initial reading and improve behavior where we
    had an error but the register become altered so we can't
    recover its previous state.
*/

bool I2C::write(uint8_t address, uint8_t reg, uint8_t data){

    while(_state != I2C_STATE_IDLE);

    uint8_t retries = I2C_WRITE_RETRIES;
    _state = I2C_STATE_BUSY;

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

        //Check if data is updated correctly
        if(verifyData == updatedData){
            _state = I2C_STATE_IDLE;
            return true;
        } 

        //Restart I2C bus before last attempt
        if(retries == 1){
            I2C1_manualRestart();
            init();
        }
    }
    
    //if we went this far, do a hardfault if necessary
    //faultHandler();
    _state = I2C_STATE_IDLE;
    return false;
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

/*
    I2C DATA READ PROCEDURE
    -------------------------------------

    In this case, because we are streaming the data we 
    are counting number of retries on the fly where
    incoming data is signaled by interrupt. (we don't
    retry immediately, only when new data is available)

    After 5 retries we restart I2C bus. Such errors can happen
    if we move IMU device in our hands when it's connected
    via a breadboard with loose cables. 

    PS. I've been testing the restart procedure, it 
    usually do the job but if I2C device hangs, only thing
    we can do is resetting whole system. (It can be further
    handled but I don't see a reason to do so here)
*/

bool I2C::read(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t n){

    //poprawić by było w pełni synchroniczne
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

bool I2C::beginRead(uint8_t address, uint8_t reg){

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

    return true;
}

bool I2C::tryRead(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t n){

    if(!beginRead(address, reg))
        return false;

    if(n == 1) {
        _i2c->CR1 &= ~I2C_CR1_ACK;
        _i2c->CR1 |= I2C_CR1_STOP;

        (void)_i2c->SR2;
        if(!waitForFlagSet(_i2c->SR1, I2C_SR1_RXNE)) return false;
        *buffer = _i2c->DR;
        return true;
    }

    _i2c->CR1 |= I2C_CR1_ACK;
    (void)_i2c->SR2;

    while (n > 0) {
        if(!waitForFlagSet(_i2c->SR1, I2C_SR1_RXNE)) return false;

        if (n == 2) {
            // Prepare NACK + STOP before reading N-1
            _i2c->CR1 &= ~I2C_CR1_ACK;
            _i2c->CR1 |= I2C_CR1_STOP;
        }

        *buffer++ = _i2c->DR;
        n--;
    }

    
    return true;
}

bool I2C::asyncRead(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t n){

    while(_state != I2C_STATE_IDLE);
    _state = I2C_STATE_BUSY;

    _packet.addr = address;
    _packet.reg = reg;
    _packet.index = 0;
    // _packet.error_counter = 0;
    // _packet.read_buffer_ptr = buffer;
    _packet.length = n;
    

    if(!beginRead(address, reg))
    {
        _state = I2C_STATE_ERROR;
        return false;
    }
        

    _i2c->CR2 |= (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

    if(_packet.length > 1){
        _i2c->CR1 |= I2C_CR1_ACK;
    } 
    else{
        _i2c->CR1 &=~ I2C_CR1_ACK;
        _i2c->CR1 |= I2C_CR1_STOP;
    }

    return true;
}

void I2C::beginAsyncRead(){

}

 void I2C::IRQhandler(){
    uint32_t SR1_tmp = _i2c->SR1;
//     uint32_t SR2_tmp = _i2c->SR2;
//    if ((SR1_tmp & I2C_SR1_BTF)) {
//         _i2c->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
//     }

//     printf("%x %x\n", SR1_tmp, SR2_tmp);

    // RXNE: regular received byte
    if (SR1_tmp & I2C_SR1_RXNE) {
        _packet.buffer[_packet.index++] = _i2c->DR;

        if (_packet.index == _packet.length - 1) {
            // volatile uint32_t t = _i2c->DR;
            _i2c->CR1 &= ~I2C_CR1_ACK; // prepare NACK
            _i2c->CR1 |= I2C_CR1_STOP;
        }

        

        if (_packet.index >= _packet.length) {
             // STOP here
              _i2c->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN );
            _state = I2C_STATE_DATA_READY;
            // volatile uint32_t t = _i2c->DR;
        }
    }
 }

void I2C::IRQerrorHandler(){

    volatile uint32_t err = _i2c->SR1;

    if(err & I2C_SR1_BERR){
        _i2c->SR1 &= ~I2C_SR1_BERR;
        _i2c->CR1 |= I2C_CR1_STOP;
    }
    if(err & I2C_SR1_ARLO){
        _i2c->SR1 &= ~I2C_SR1_ARLO;
        _i2c->CR1 |= I2C_CR1_STOP;
    }
    if(err & I2C_SR1_AF){
        _i2c->SR1 &= ~I2C_SR1_AF;
        _i2c->CR1 |= I2C_CR1_STOP;
    }
    if(err & I2C_SR1_OVR){
        _i2c->SR1 &= ~I2C_SR1_OVR;
        _i2c->CR1 |= I2C_CR1_STOP;
    }
    if(err & I2C_SR1_TIMEOUT){
        _i2c->SR1 &= ~I2C_SR1_TIMEOUT;
        _i2c->CR1 |= I2C_CR1_STOP;
    }

    _i2c->CR2 &=~ (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    _i2c->CR1 |= I2C_CR1_STOP;
    _state = I2C_STATE_ERROR;
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

uint8_t I2C::checkErrors(uint16_t timeout) {
    
    if(!timeout){
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_TIMEOUT;
    }
    else if(_i2c->SR1 & I2C_SR1_BERR){
        _i2c->SR1 &= ~I2C_SR1_BERR;
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_BERR; 
    }
    else if(_i2c->SR1 & I2C_SR1_ARLO){
        _i2c->SR1 &= ~I2C_SR1_ARLO;
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_ARLO; 
    }
    else if(_i2c->SR1 & I2C_SR1_AF){
        _i2c->SR1 &= ~I2C_SR1_AF;
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_AF;
    }
    else if(_i2c->SR1 & I2C_SR1_OVR){
        _i2c->SR1 &= ~I2C_SR1_OVR;
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_OVR;
    }
    else if(_i2c->SR1 & I2C_SR1_TIMEOUT){
        _i2c->SR1 &= ~I2C_SR1_TIMEOUT;
        _i2c->CR1 |= I2C_CR1_STOP;
        return I2C_ERROR_TIMEOUT;
    }

    return I2C_OK;
}







// bool I2C::read(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t n){

//     if(_packet.error_counter == I2C_READ_RETRIES){
//         I2C1_manualRestart();
//         init();
//         _packet.error_counter = 0;
//     }

//     if(!tryRead(address, reg, buffer, n)){
//         _packet.error_counter++;

//         if(_packet.error_counter == I2C_READ_RETRIES){
//             I2C1_manualRestart();
//             init();
//             _packet.error_counter = 0;
//         }

//         printf("Error\n");
//         return false;
//     }
        
//     return true;
// }


// bool I2C::tryAsyncRead(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t n){

    // while(_state != I2C_STATE_IDLE);


    // if(async){
    //     _packet.addr = address;
    //     _packet.reg = reg;
    //     _packet.index = 0;
    //     // _packet.error_counter = 0;
    //     _packet.read_buffer_ptr = buffer;
    //     _packet.length = n;
    //     beginAsyncRead();
    //     while(_state == I2C_STATE_BUSY);
    //     return true;
    // }


    // while(_state != I2C_STATE_IDLE);

    // _state = I2C_STATE_BUSY;
   
    // if(!waitForFlagClear(_i2c->SR2, I2C_SR2_BUSY)) return false;
    //  _i2c->CR1 &= ~I2C_CR1_STOP;
    // _i2c->CR1 |= I2C_CR1_START;

    
  

    // if(!waitForFlagSet(_i2c->SR1, I2C_SR1_SB)) return false;
    // _i2c->DR = address << 1;

    // if(!waitForFlagSet(_i2c->SR1, I2C_SR1_ADDR)) return false;
    // (void)_i2c->SR2;

    // if(!waitForFlagSet(_i2c->SR1, I2C_SR1_TXE)) return false;
    // _i2c->DR = reg;

    // if(!waitForFlagSet(_i2c->SR1, I2C_SR1_TXE)) return false;
    
    // _i2c->CR1 |= I2C_CR1_START;

    // if(!waitForFlagSet(_i2c->SR1, I2C_SR1_SB)) return false;
    // _i2c->DR = address << 1 | 1;

    // if(!waitForFlagSet(_i2c->SR1, I2C_SR1_ADDR)) return false;
    // (void)_i2c->SR2;

    // if(async){
    //     _packet.addr = address;
    //     _packet.reg = reg;
    //     _packet.index = 0;
    //     _packet.error_counter = 0;
    //     _packet.read_buffer_ptr = buffer;
    //     _packet.length = n;

    //     _i2c->CR2 |= (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN);

    //     if(n > 1){
    //         _i2c->CR1 |= I2C_CR1_ACK;
    //     } 
    //     else{
    //         _i2c->CR1 &= ~I2C_CR1_ACK;
    //         _i2c->CR1 |= I2C_CR1_STOP;
    //     }

    //      while(_state != I2C_STATE_IDLE);
    //     return true;
    // }
  
    // // if(!waitForFlagClear(_i2c->SR2, I2C_SR2_BUSY)) return false;  
    // _i2c->CR1 |= I2C_CR1_ACK;

    // while (n > 0) {
    //     if(!waitForFlagSet(_i2c->SR1, I2C_SR1_RXNE)) return false;

    //     if (n == 1) {
    //         _i2c->CR1 &= ~I2C_CR1_ACK;
    //         _i2c->CR1 |= I2C_CR1_STOP;
    //     }

    //     *buffer++ = _i2c->DR;
    //     n--;
    // }
     
    // _state = I2C_STATE_IDLE;
    // return true;
// }
