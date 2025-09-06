#pragma once

#include "stm32f446xx.h"
#include "i2c_utils.h"

#define SYS_CLK 16000000
#define PCLK1   SYS_CLK
#define I2C_FREQ 400000
#define I2C_FAST_MODE_MAX_RISE_TIME 300

#define I2C_WRITE_RETRIES 5
#define I2C_READ_RETRIES 5
#define I2C_TIMEOUT_VAL 20000

enum{
    I2C_OK,
    I2C_ERROR_TIMEOUT,
    I2C_ERROR_BERR,
    I2C_ERROR_ARLO,
    I2C_ERROR_AF,
    I2C_ERROR_OVR,
};

enum I2Cstate{
    I2C_STATE_IDLE,
    I2C_STATE_BUSY,
    I2C_STATE_WRITE_REG,
    I2C_STATE_READ
};

typedef struct{
    uint8_t write_address;
    uint8_t read_address;
    uint8_t reg;
    uint8_t* buffer_ptr;
    uint8_t length;
    uint8_t index;
    uint8_t error_counter;
} I2C_read_packet;

class I2C {
public:
    I2C(I2C_TypeDef* i2c);
    void init();
    bool write(uint8_t address, uint8_t reg, uint8_t data);
    bool read(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t n);
    void readIRQhandler();
private:
    bool tryRead(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t n);
    bool tryWrite(uint8_t address, uint8_t reg, uint8_t data);
    bool waitForFlagSet(volatile uint32_t& reg, uint32_t flag);
    bool waitForFlagClear(volatile uint32_t& reg, uint32_t flag);
    uint8_t checkErrors(uint16_t timeout);
    I2C_TypeDef* _i2c;
    uint8_t _state;
    I2C_read_packet _packet;
};