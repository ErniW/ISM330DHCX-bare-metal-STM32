#include "ism330.h"
#include "systick.h"
#include "timer.h"
#include "Fusion/Fusion.h"

ISM330DHCX::ISM330DHCX(uint8_t address, I2C* i2c) : 
    _address(address),
    _i2c(i2c),
    accelFilterX(0.0f, 1.0f, 0.001f, 0.1f),
    accelFilterY(0.0f, 1.0f, 0.001f, 0.1f),
    accelFilterZ(0.0f, 1.0f, 0.001f, 0.1f)
{}

void ISM330DHCX::init(uint8_t accelFreq, uint8_t accelRange, uint8_t gyroFreq, uint8_t gyroDPS){

    _i2c->write(_address, CTRL3_C, SW_RESET);
    delay_ms(100);
    // _i2c->write(_address, CTRL3_C, AUTO_INC);

    _i2c->write(_address, CTRL1_XL, (accelFreq << 4) | (accelRange << 2));
    _i2c->write(_address, CTRL2_G, (gyroFreq << 4) | gyroDPS);   

    accelSensitivity = getAccelSensitivity(accelRange);
    gyroSensitivity = getGyroSensitivity(gyroDPS);
    
    timerInit();
    timerEnable();

    FusionAhrsInitialise(&ahrs);
    FusionOffsetInitialise(&offset, 416);
}

void ISM330DHCX::gyroInterruptEnable()
{
    _i2c->write(_address, COUNTER_BDR_REG1, DRDY_PULSE);
    _i2c->write(_address, INT1_CTRL, INT1_DRDY_G);
 
}

bool ISM330DHCX::accelReadBlocking(int16_t& x, int16_t& y, int16_t& z){

    uint8_t buffer[6] = {0};

    _i2c->read(_address, READ_ACCEL, buffer, 6);

    // while(_i2c->_state == I2C_STATE_BUSY);

    x = (buffer[1] << 8 | buffer[0]);
    y = (buffer[3] << 8 | buffer[2]);
    z = (buffer[5] << 8 | buffer[4]);

    return true;
}


bool ISM330DHCX::gyroReadBlocking(int16_t& x, int16_t& y, int16_t& z){

    uint8_t buffer[6] = {0};

    _i2c->read(_address, READ_GYRO, buffer, 6);

    x = (buffer[1] << 8 | buffer[0]);
    y = (buffer[3] << 8 | buffer[2]);
    z = (buffer[5] << 8 | buffer[4]);

    return true;
}

bool ISM330DHCX::gyroRequest(){
    if(!_i2c->asyncRead(_address, READ_GYRO, _i2c->_packet.buffer, 6))
        return false;

    _state = IMU_STATE_WAIT_FOR_GYRO_DATA;
    return true;
}

bool ISM330DHCX::accelRequest(){
    if(!_i2c->asyncRead(_address, READ_ACCEL, _i2c->_packet.buffer, 6))
        return false;
        
    _state = IMU_STATE_WAIT_FOR_ACCEL_DATA;
    return true;
}

void ISM330DHCX::gyroDataReadyHandler(){
    gx = (_i2c->_packet.buffer[1] << 8 | _i2c->_packet.buffer[0]);
    gy = (_i2c->_packet.buffer[3] << 8 | _i2c->_packet.buffer[2]);
    gz = (_i2c->_packet.buffer[5] << 8 | _i2c->_packet.buffer[4]);
}

void ISM330DHCX::accelDataReadyHandler(){
    ax = (_i2c->_packet.buffer[1] << 8 | _i2c->_packet.buffer[0]);
    ay = (_i2c->_packet.buffer[3] << 8 | _i2c->_packet.buffer[2]);
    az = (_i2c->_packet.buffer[5] << 8 | _i2c->_packet.buffer[4]);
}

void ISM330DHCX::gyroInterruptHandler(){
    dt = getDt(timerGetTime());
    _state = IMU_STATE_GET_GYRO_DATA;
    isGyroDataReady = false;   
}

void ISM330DHCX::gyroCalibrate(uint16_t samples){
    float avgX = 0; 
    float avgY = 0;
    float avgZ = 0;

    delay_ms(100);
    for(uint16_t i=0; i<samples; i++){
        int16_t gx, gy, gz = 0;
        gyroReadBlocking(gx, gy, gz);
        avgX += (float)gx;
        avgY += (float)gy;
        avgZ += (float)gz;
        delay_ms(10);
    }

    gyroCalibrationX = avgX / samples;
    gyroCalibrationY = avgY / samples;
    gyroCalibrationZ = avgZ / samples;
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
            return GYRO_SENSITIVITY_125 * SEC_TO_MS;
        case GYRO_250_DPS:
            return GYRO_SENSITIVITY_250 * SEC_TO_MS;
        case GYRO_500_DPS:
            return GYRO_SENSITIVITY_500 * SEC_TO_MS;
        case GYRO_1000_DPS:
            return GYRO_SENSITIVITY_1000 * SEC_TO_MS;
        case GYRO_2000_DPS:
            return GYRO_SENSITIVITY_2000 * SEC_TO_MS;
        case GYRO_4000_DPS:
            return GYRO_SENSITIVITY_4000 * SEC_TO_MS;
        default:
            return GYRO_SENSITIVITY_125 * SEC_TO_MS;
    }
}

float ISM330DHCX::getDt(uint32_t timestamp){
    uint32_t delta = 0;

    if(timestamp >= timestampLast)
        delta = timestamp - timestampLast;
    else
        delta = (0xFFFFFFFF - timestampLast + 1) + timestamp;

    timestampLast = timestamp;

    return (delta / 1000000.0f);
}

/*
    GET IMU ORIENTATION
    -------------------------------------

    - Fusion library (formerly called Madgwick).
    - Custom timer dedicated to get delta time.
    - 1D kalman filter on accelerometer data.
    - Drop invalid data and noise below threshold.
    - Quaternion based.
*/
void ISM330DHCX::computeIMU() {

    FusionVector gyroscope = {
        (float)gx - gyroCalibrationX, 
        (float)gy - gyroCalibrationY,
        (float)gz - gyroCalibrationZ
    };

    if(fabs(gyroscope.axis.x) < GYRO_NOISE_THRESHOLD) gyroscope.axis.x = 0.0f;
    if(fabs(gyroscope.axis.y) < GYRO_NOISE_THRESHOLD) gyroscope.axis.y = 0.0f;
    if(fabs(gyroscope.axis.z) < GYRO_NOISE_THRESHOLD) gyroscope.axis.z = 0.0f;

    gyroscope.axis.x *= gyroSensitivity;
    gyroscope.axis.y *= gyroSensitivity;
    gyroscope.axis.z *= gyroSensitivity;

    FusionVector accelerometer = {
        ax * accelSensitivity,
        ay * accelSensitivity,
        az * accelSensitivity
    };

    accelerometer.axis.x = accelFilterX.update(accelerometer.axis.x);
    accelerometer.axis.y = accelFilterY.update(accelerometer.axis.y);
    accelerometer.axis.z = accelFilterZ.update(accelerometer.axis.z);

    gyroscope = FusionOffsetUpdate(&offset, gyroscope);
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);
    quaternion = FusionAhrsGetQuaternion(&ahrs);
};



void ISM330DHCX::pedometerEnable(){
    _i2c->write(_address, FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_EN);
    _i2c->write(_address, EMB_FUNC_EN_A, PEDO_EN);
    _i2c->write(_address, FUNC_CFG_ACCESS, 0x00);
}

uint16_t ISM330DHCX::pedometeRead(){

    uint16_t steps = 0;

    _i2c->write(_address, FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_EN);
    _i2c->read(_address, EMB_FUNC_STEP_COUNTER_L, (uint8_t*)&steps, 2);
    _i2c->write(_address, FUNC_CFG_ACCESS, 0x00);

    return steps;
}

void ISM330DHCX::singleTapEnable(){
    _i2c->write(_address, TAP_CFG0, INT_CLR_ON_READ | TAP_X_EN | TAP_Y_EN | TAP_Z_EN);
    _i2c->write(_address, TAP_CFG1, TAP_THRESHOLD_X);
    _i2c->write(_address, TAP_CFG2, INTERRUPTS_EN | TAP_THRESHOLD_Y);
    _i2c->write(_address, TAP_THS_6D, TAP_THRESHOLD_Z);
    _i2c->write(_address, INT_DUR2, TAP_SHOCK | TAP_QUIET);
}

uint8_t ISM330DHCX::singleTapRead(){
     uint8_t state;

    _i2c->read(_address, TAP_SRC, &state, 1);

    if(!(state & SINGLE_TAP))
        return TAP_NO_EVENT;
        
    state &= 0xF;
    return state;
}

volatile uint8_t ISM330DHCX::getState(){
    return _state;
}

void ISM330DHCX::setState(uint8_t state){
    _state = state;
}

bool ISM330DHCX::gyroIsAvailable(){
    return isGyroDataReady;
}

void ISM330DHCX::gyroSetDataAvailable(){
    isGyroDataReady = true;
}