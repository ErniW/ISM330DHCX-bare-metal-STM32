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

        // _i2c->CR1 &= ~I2C_CR1_PE;
        // _i2c->CR1 |= I2C_CR1_SWRST;
        // _i2c->CR1 &=~ I2C_CR1_SWRST;

        // // Toggle the peripheral reset bit in RCC
        // if (_i2c == I2C1) {
        //     RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        //     RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
        // } else if (_i2c == I2C2) {
        //     RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
        //     RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
        // }
        // // etc., for I2C3 if used

        // // Re-enable the peripheral
        // _i2c->CR1 |= I2C_CR1_PE;
        // _i2c->CR1 |= I2C_CR1_STOP;
        // for(volatile long i=0; i<100000; i++);

 // Disable I2C1
    I2C1->CR1 &= ~I2C_CR1_PE;

    // 2. Force and release reset via RCC
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    for (volatile int i = 0; i < 1000; ++i); // short delay
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    // 3. Enable GPIOB clock (if not already)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // 4. Configure PB8 (SCL) and PB9 (SDA) as open-drain outputs
    GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |= (1 << GPIO_MODER_MODE8_Pos) | (1 << GPIO_MODER_MODE9_Pos); // Output mode
    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9; // Open-drain
    GPIOB->ODR |= GPIO_ODR_OD8 | GPIO_ODR_OD9; // Drive both lines high

    // Short delay
    for (volatile int i = 0; i < 1000; ++i);

    // 5. Clock SCL 9 times while checking SDA
    for (int i = 0; i < 9; i++) {
        if (GPIOB->IDR & GPIO_IDR_ID9) break; // SDA released
        GPIOB->ODR &= ~GPIO_ODR_OD8; // SCL low
        for (volatile int d = 0; d < 1000; ++d);
        GPIOB->ODR |= GPIO_ODR_OD8; // SCL high
        for (volatile int d = 0; d < 1000; ++d);
    }

    // 6. Issue a STOP condition
    GPIOB->ODR &= ~GPIO_ODR_OD9; // SDA low
    for (volatile int d = 0; d < 1000; ++d);
    GPIOB->ODR |= GPIO_ODR_OD8;  // SCL high
    for (volatile int d = 0; d < 1000; ++d);
    GPIOB->ODR |= GPIO_ODR_OD9;  // SDA high
    for (volatile int d = 0; d < 1000; ++d);

    // 7. Reconfigure PB8/9 for I2C1 (AF4)
    GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |= (2 << GPIO_MODER_MODE8_Pos) | (2 << GPIO_MODER_MODE9_Pos); // AF mode
    GPIOB->AFR[1] &= ~((0xF << GPIO_AFRH_AFSEL8_Pos) | (0xF << GPIO_AFRH_AFSEL9_Pos));
    GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL8_Pos) | (4 << GPIO_AFRH_AFSEL9_Pos); // AF4 = I2C1

    // 8. Re-enable I2C1
    I2C1->CR1 |= I2C_CR1_PE;

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