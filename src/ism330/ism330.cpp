#include "ism330.h"

ISM330DHCX::ISM330DHCX(uint8_t address, I2C* i2c) : _address(address), _i2c(i2c) {};

void ISM330DHCX::init(uint8_t accelFreq, uint8_t accelSensitivity, uint8_t gyroFreq, uint8_t gyroDPS){

    _i2c->write(ADDRESS, CTRL3_C, (SW_RESET | AUTO_INC));

    _i2c->write(ADDRESS, CTRL1_XL, (accelFreq << 4) | (accelSensitivity << 2));
    _i2c->write(ADDRESS, CTRL2_G, (gyroFreq << 4) | gyroDPS);   
}

void ISM330DHCX::readAccel(int16_t& x, int16_t& y, int16_t& z){

    uint8_t buffer[6];

    _i2c->read(ADDRESS, READ_ACCEL, buffer, 6);

    x = (buffer[1] << 8 | buffer[0]);
    y = (buffer[3] << 8 | buffer[2]);
    z = (buffer[5] << 8 | buffer[4]);
}


void ISM330DHCX::readGyro(int16_t& x, int16_t& y, int16_t& z){

    uint8_t buffer[6];

    _i2c->read(ADDRESS, READ_GYRO, buffer, 6);

    x = (buffer[1] << 8 | buffer[0]);
    y = (buffer[3] << 8 | buffer[2]);
    z = (buffer[5] << 8 | buffer[4]);
}

void ISM330DHCX::getIMU(float& roll, float& pitch, float& yaw){

    int16_t ax, ay, az, gx, gy, gz;

    readAccel(ax, ay, az);
    readGyro(gx, gy, gz);

    roll = atan2f(ay, az) * (180.0f / M_PI);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);

    float gyro_roll_rate = gy / SAMPLE_RATE;
    float gyro_pitch_rate = gx / SAMPLE_RATE;

    roll += gyro_roll_rate;
    pitch += gyro_pitch_rate;

    roll = ACCELEROMETER_GAIN * roll + GYROSCOPE_GAIN * roll;
    pitch = ACCELEROMETER_GAIN * pitch + GYROSCOPE_GAIN * pitch;


    yaw += gz / SAMPLE_RATE;
    yaw = ACCELEROMETER_GAIN * yaw + GYROSCOPE_GAIN * yaw;
    yaw = fmodf(yaw, 360);
}

void ISM330DHCX::enablePedometer(){

    //tutaj musiałbym odczytywać wartość a następnie ustawić.

    _i2c->write(_address, FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_EN);
    _i2c->write(_address, EMB_FUNC_EN_A, PEDO_EN);
    _i2c->write(_address, FUNC_CFG_ACCESS, 0x00);
}

void ISM330DHCX::readPedometer(int16_t& steps){

    _i2c->write(_address, FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_EN);


    uint8_t buffer[2];

    _i2c->read(_address, EMB_FUNC_STEP_COUNTER_L, buffer, 2);

    steps = (buffer[1] << 8 | buffer[0]);

    _i2c->write(_address, FUNC_CFG_ACCESS, 0x00);

}

void ISM330DHCX::enableSingleTap(){

    _i2c->write(_address, TAP_CFG0, INT_CLR_ON_READ | TAP_X_EN | TAP_Y_EN | TAP_Z_EN);
    _i2c->write(_address, TAP_CFG1, TAP_THRESHOLD_X);
    _i2c->write(_address, TAP_CFG2, INTERRUPTS_EN | TAP_THRESHOLD_Y);
    _i2c->write(_address, TAP_THS_6D, TAP_THRESHOLD_Z);

    _i2c->write(_address, INT_DUR2, TAP_SHOCK | TAP_QUIET);
}

void ISM330DHCX::readSingleTap(volatile uint8_t& tap){
     uint8_t buffer[1];

    _i2c->read(_address, TAP_SRC, buffer, 1);

    tap = buffer[0];

}