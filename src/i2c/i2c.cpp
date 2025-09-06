#include "i2c.h"
#include <string.h>
#include <cstdio>

I2C::I2C (I2C_TypeDef* i2c) : _i2c(i2c) {};

void I2C::init(){
    _i2c->CR1 &=~ I2C_CR1_PE;
    _i2c->CR1 |= I2C_CR1_SWRST;
    _i2c->CR1 &=~ I2C_CR1_SWRST;
    _i2c->CR2 |= (PCLK1 / 1000000);
    _i2c->CCR = PCLK1 / (3 * I2C_FREQ) | I2C_CCR_FS;
    _i2c->TRISE = (I2C_FAST_MODE_MAX_RISE_TIME * (PCLK1 / 1000000))/1000 + 1;
    
    // _i2c->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);

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

    _i2c->CR1 |= I2C_CR1_PE;

    _state = I2C_STATE_IDLE;
    _packet = {0};


}

void I2C::write(uint8_t address, uint8_t reg, uint8_t value){
    _state = I2C_STATE_BUSY;
    _packet.operation = I2C_WRITE;
    _packet.step = I2C_STEP_WRITE_ADDR;

    _packet.addr = address;
    _packet.reg = reg;
    _packet.write_value = value;

    _i2c->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    _i2c->CR1 |= I2C_CR1_START;

    while(_i2c->SR2 & I2C_SR2_BUSY);
}

void I2C::read(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t length) {
    _state = I2C_STATE_BUSY;
    _packet.operation = I2C_READ;
    _packet.step = I2C_STEP_WRITE_ADDR;

    _packet.addr = 
}

void I2C::IRQhandler() {
    uint32_t SR1_tmp = _i2c->SR1;

    if(SR1_tmp  & I2C_SR1_SB){
        if(_packet.step == I2C_STEP_WRITE_ADDR){
            _i2c->DR = (_packet.addr << 1) | 0;
        }
        else if(_packet.operation == I2C_STEP_WRITE_READ_ADDR){
            _i2c->DR = (_packet.addr << 1) | 1;
        }     
    }
    if(SR1_tmp  & I2C_SR1_ADDR)
    {
        uint32_t SR2_tmp = _i2c->SR2;
         (void)_i2c->SR2;
         _packet.step = I2C_STEP_WRITE_REG;
    }
    // if(SR1_tmp  & I2C_SR1_BTF)
    // {
    //     // if(_packet.step == I2C_STEP_WAIT_BTF)
    //     // {
    //     //     _i2c->CR1 |= I2C_CR1_STOP;
    //     //     _i2c->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    //     //     _state = I2C_STATE_DONE;
    //     // }
    // }
    if(SR1_tmp  & I2C_SR1_TXE)
    {
        if(_packet.step == I2C_STEP_WRITE_REG)
        {
            _i2c->DR = _packet.reg;
            _packet.step = I2C_STEP_WRITE_VAL;
        }
        else if(_packet.step == I2C_STEP_WRITE_VAL)
        {
            _i2c->DR = _packet.write_value;
            _i2c->CR1 |= I2C_CR1_STOP;
            _i2c->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
            _state = I2C_STATE_DONE;
        }
    }



}

// void I2C::IRQhandler() {
//     uint32_t SR1_tmp = _i2c->SR1;
//     uint32_t SR2_tmp;

//     // START sent → send address
//     if (SR1_tmp & I2C_SR1_SB) {
//         _i2c->DR = (_packet.addr << 1) | (_packet.operation == I2C_READ ? 1 : 0);
//     }
//     // ADDR matched → clear ADDR
//     else if (SR1_tmp & I2C_SR1_ADDR) {
//         SR2_tmp = _i2c->SR2;
//         (void)SR2_tmp;

//         // Single-byte read: disable ACK
//         if (_packet.operation == I2C_READ && _packet.length == 1)
//             _i2c->CR1 &= ~I2C_CR1_ACK;
//     }

//     // TXE → write register first, then data
//     if ((SR1_tmp & I2C_SR1_TXE) && _packet.operation == I2C_WRITE) {
//         if (!(_i2c->SR1 & I2C_SR1_BTF)) {  // first TXE → send register
//             _i2c->DR = _packet.reg;
//         } else {                           // next BTF → send value
//             _i2c->DR = _packet.write_value;
//         }
//     }

//     // RXNE → read data into buffer
//     if ((SR1_tmp & I2C_SR1_RXNE) && _packet.operation == I2C_READ) {
//         _packet.read_buffer_ptr[_packet.index++] = _i2c->DR;

//         // Disable ACK before last byte
//         // if (_packet.index == _packet.length - 1)
//         //     _i2c->CR1 &= ~I2C_CR1_ACK;

//         if (_packet.index >= _packet.length) {
//             _i2c->CR1 |= I2C_CR1_STOP;
//             _i2c->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
//             _state = I2C_STATE_DONE;
//         }
//     }

//     // BTF → stop condition if done
//     if (SR1_tmp & I2C_SR1_BTF) {
//         if ((_packet.operation == I2C_WRITE) ||
//             (_packet.operation == I2C_READ && _packet.index >= _packet.length)) 
//         {
//             _i2c->CR1 |= I2C_CR1_STOP;
//             _i2c->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
//             _state = I2C_STATE_DONE;
//         }
//     }
// }

