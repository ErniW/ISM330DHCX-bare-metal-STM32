#include "ism330.h"
#include <memory>

ISM330DHCX::ISM330DHCX(uint8_t address, I2C* i2c) : _address(address),_i2c(i2c) {};

void ISM330DHCX::init(){
    _i2c->write(ADDRESS, CTRL3_C, (SW_RESET | AUTO_INC));
    _i2c->write(ADDRESS, CTRL1_XL, ACCELEROMETER_416HZ_2G);
    _i2c->write(ADDRESS, CTRL2_G, GYROSCOPE_416HZ_2000DPS);
}

void ISM330DHCX::readAccel(int16_t& x, int16_t& y, int16_t& z){

    char buffer[6];

    _i2c->read(ADDRESS, READ_ACCEL, (char*)buffer, 6);

    x = (buffer[1] << 8 | buffer[0]);
    y = (buffer[3] << 8 | buffer[2]);
    z = (buffer[5] << 8 | buffer[4]);
}


void ISM330DHCX::readGyro(int16_t& x, int16_t& y, int16_t& z){

    char buffer[6];

    _i2c->read(ADDRESS, READ_GYRO, (char*)buffer, 6);

    x = (buffer[1] << 8 | buffer[0]);
    y = (buffer[3] << 8 | buffer[2]);
    z = (buffer[5] << 8 | buffer[4]);
}