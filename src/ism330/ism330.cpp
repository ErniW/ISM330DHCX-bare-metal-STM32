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



#define ACCELEROMETER_SENSITIVITY   0.061f  // mg/LSB
#define GYROSCOPE_SENSITIVITY       0.070f* 2.0f  // dps/LSB for ±2000 dps
#define SAMPLE_RATE                 102.0f  // Hz

#define ACCELEROMETER_GAIN          0.98f
#define GYROSCOPE_GAIN              0.02f

void ISM330DHCX::getIMU(float& roll, float& pitch, float& yaw) {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    // Read raw sensor values
    readAccel(ax_raw, ay_raw, az_raw);
    readGyro(gx_raw, gy_raw, gz_raw);

    // Convert accelerometer to g (assuming it's in mg/LSB)
    float ax = ax_raw * ACCELEROMETER_SENSITIVITY / 1000.0f;
    float ay = ay_raw * ACCELEROMETER_SENSITIVITY / 1000.0f;
    float az = az_raw * ACCELEROMETER_SENSITIVITY / 1000.0f;

    // Compute roll/pitch from accelerometer
    float accel_roll  = atan2f(ay, az) * (180.0f / M_PI);
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);

    // Convert gyro to dps
    float gyro_roll_rate  = gy_raw * GYROSCOPE_SENSITIVITY;
    float gyro_pitch_rate = gx_raw * GYROSCOPE_SENSITIVITY;
    float gyro_yaw_rate   = gz_raw * GYROSCOPE_SENSITIVITY;

    // Integrate gyro over time
    roll  += gyro_roll_rate  * (1.0f / SAMPLE_RATE);
    pitch += gyro_pitch_rate * (1.0f / SAMPLE_RATE);

    // Apply complementary filter (gyro + accelerometer)
    roll  = ACCELEROMETER_GAIN * accel_roll  + GYROSCOPE_GAIN * roll;
    pitch = ACCELEROMETER_GAIN * accel_pitch + GYROSCOPE_GAIN * pitch;

    // Adjust yaw sensitivity slightly (reduce by 10%, instead of 90%)
    // gyro_yaw_rate *= 0.01f;  // Reduce yaw sensitivity by 10%

    // Integrate yaw
    yaw += gyro_yaw_rate * (1.0f / SAMPLE_RATE);

    // Low-pass filter to smooth yaw
    // float new_yaw = yaw + gyro_yaw_rate * (1.0f / SAMPLE_RATE);
    // yaw = 0.98f * yaw + 0.02f * new_yaw;

    // Wrap yaw to [0, 360)
    if (yaw < 0) yaw += 360;
    else if (yaw >= 360) yaw -= 360;
}



void ISM330DHCX::enablePedometer(){

    //tutaj musiałbym odczytywać wartość a następnie ustawić.

    _i2c->write(_address, FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_EN);
    _i2c->write(_address, EMB_FUNC_EN_A, PEDO_EN);
    _i2c->write(_address, FUNC_CFG_ACCESS, 0x00);
}

uint16_t ISM330DHCX::readPedometer(){

    uint16_t steps = 0;

    _i2c->write(_address, FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_EN);

    _i2c->read(_address, EMB_FUNC_STEP_COUNTER_L, (uint8_t*)&steps, 2);

    _i2c->write(_address, FUNC_CFG_ACCESS, 0x00);

    return steps;

}

void ISM330DHCX::enableSingleTap(){

    _i2c->write(_address, TAP_CFG0, INT_CLR_ON_READ | TAP_X_EN | TAP_Y_EN | TAP_Z_EN);
    _i2c->write(_address, TAP_CFG1, TAP_THRESHOLD_X);
    _i2c->write(_address, TAP_CFG2, INTERRUPTS_EN | TAP_THRESHOLD_Y);
    _i2c->write(_address, TAP_THS_6D, TAP_THRESHOLD_Z);
    _i2c->write(_address, INT_DUR2, TAP_SHOCK | TAP_QUIET);
}

uint8_t ISM330DHCX::readSingleTap(){
     uint8_t state;

    _i2c->read(_address, TAP_SRC, &state, 1);

    if(!(state & SINGLE_TAP))
        return TAP_NO_EVENT;
        
    state &= 0xF;
    return state;
}