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



#define ACCELEROMETER_SENSITIVITY 0.061f
#define GYROSCOPE_SENSITIVITY 8.75f
#define SAMPLE_RATE 416/10

#define ACCELEROMETER_GAIN 0.02f 
#define GYROSCOPE_GAIN 0.98f



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