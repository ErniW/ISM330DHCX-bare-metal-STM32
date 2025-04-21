#include "ism330.h"

ISM330DHCX::ISM330DHCX(uint8_t address, I2C* i2c) : _address(address), _i2c(i2c) {};

void ISM330DHCX::init(uint8_t accelFreq, uint8_t accelRange, uint8_t gyroFreq, uint8_t gyroDPS){

    _i2c->write(_address, CTRL3_C, (SW_RESET | AUTO_INC));

    _i2c->write(_address, CTRL1_XL, (accelFreq << 4) | (accelRange << 2));
    _i2c->write(_address, CTRL2_G, (gyroFreq << 4) | gyroDPS);   

    accelSensitivity = getAccelSensitivity(accelRange);
    gyroSensitivity = getGyroSensitivity(gyroDPS);
}

void ISM330DHCX::readAccel(int16_t& x, int16_t& y, int16_t& z){

    uint8_t buffer[6];

    _i2c->read(_address, READ_ACCEL, buffer, 6);

    x = (buffer[1] << 8 | buffer[0]);
    y = (buffer[3] << 8 | buffer[2]);
    z = (buffer[5] << 8 | buffer[4]);
}


void ISM330DHCX::readGyro(int16_t& x, int16_t& y, int16_t& z){

    uint8_t buffer[6];

    _i2c->read(_address, READ_GYRO, buffer, 6);

    x = (buffer[1] << 8 | buffer[0]);
    y = (buffer[3] << 8 | buffer[2]);
    z = (buffer[5] << 8 | buffer[4]);
}

float ISM330DHCX::getAccelSensitivity(uint8_t range){
    switch(range){
        case ACCEL_2G:
            return ACCEL_SENSITIVITY_2G;
        case ACCEL_4G:
            return ACCEL_SENSITIVITY_4G;
        case ACCEL_8G:
            return ACCEL_SENSITIVITY_8G;
        case ACCEL_16G:
            return ACCEL_SENSITIVITY_16G;   
        default:
            return ACCEL_SENSITIVITY_2G;
    }
}

float ISM330DHCX::getGyroSensitivity(uint8_t range){
    switch(range){
        case GYRO_125_DPS:
            return GYRO_SENSITIVITY_125;
        case GYRO_250_DPS:
            return GYRO_SENSITIVITY_250;
        case GYRO_500_DPS:
            return GYRO_SENSITIVITY_500;
        case GYRO_1000_DPS:
            return GYRO_SENSITIVITY_1000;
        case GYRO_2000_DPS:
            return GYRO_SENSITIVITY_2000;
        case GYRO_4000_DPS:
            return GYRO_SENSITIVITY_4000;
        default:
            return GYRO_SENSITIVITY_125;
    }
}

#define ACCELEROMETER_GAIN          0.98f
#define GYROSCOPE_GAIN              0.02f
#define SAMPLE_RATE                 100

void ISM330DHCX::getIMU(float& roll, float& pitch, float& yaw) {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    readAccel(ax_raw, ay_raw, az_raw);
    readGyro(gx_raw, gy_raw, gz_raw);

    float ax = ax_raw * (accelSensitivity / 1000);
    float ay = ay_raw * (accelSensitivity / 1000);
    float az = az_raw * (accelSensitivity / 1000);

    float gyro_roll_rate  = gy_raw * (gyroSensitivity / 1000);
    float gyro_pitch_rate = gx_raw * (gyroSensitivity / 1000);
    float gyro_yaw_rate   = gz_raw * (gyroSensitivity / 1000);

    float accel_roll  = atan2f(ay, az) * (180.0f / M_PI);
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);

    roll  += gyro_roll_rate  * (1.0f / 100);
    pitch += gyro_pitch_rate * (1.0f / 100);

    // Apply complementary filter (gyro + accelerometer)
    roll  = ACCELEROMETER_GAIN * accel_roll  + GYROSCOPE_GAIN * roll;
    pitch = ACCELEROMETER_GAIN * accel_pitch + GYROSCOPE_GAIN * pitch;

    // Adjust yaw sensitivity slightly (reduce by 10%, instead of 90%)
    // gyro_yaw_rate *= 0.005f;  // Reduce yaw sensitivity by 10%

    // Integrate yaw
    yaw += gyro_yaw_rate * (1.0f / 104);


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