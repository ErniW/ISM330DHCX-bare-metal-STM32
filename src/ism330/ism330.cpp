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

bool ISM330DHCX::readAccel(int16_t& x, int16_t& y, int16_t& z){

    uint8_t buffer[6] = {0};

    _i2c->read(_address, READ_ACCEL, buffer, 6);
    while(_i2c->_state != I2C_STATE_IDLE);
    x = (buffer[1] << 8 | buffer[0]);
    y = (buffer[3] << 8 | buffer[2]);
    z = (buffer[5] << 8 | buffer[4]);

    printf("%d, %d, %d\n", x, y, z);

    return true;
}


bool ISM330DHCX::readGyro(int16_t& x, int16_t& y, int16_t& z){

    uint8_t buffer[6] = {0};

    _i2c->read(_address, READ_GYRO, buffer, 6);

    x = (buffer[1] << 8 | buffer[0]);
    y = (buffer[3] << 8 | buffer[2]);
    z = (buffer[5] << 8 | buffer[4]);

    return true;
}

void ISM330DHCX::calibrateGyro(uint16_t samples){
    float avgX = 0; 
    float avgY = 0;
    float avgZ = 0;

    for(uint16_t i=0; i<samples; i++){
        int16_t gx,gy,gz = 0;
        readGyro(gx,gy,gz);
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

float ISM330DHCX::getDt(uint8_t frequency){
    switch(frequency){
        case FREQ_12_5_HZ:
            return 1.0 / 12.5;
        case FREQ_26_HZ:
            return 1.0 / 26.0;
        case FREQ_52_HZ:
            return 1.0 / 52.0;
        case FREQ_104_HZ:
            return 1.0 / 104.0;
        case FREQ_208_HZ:
            return 1.0 / 208.0;
        case FREQ_416_HZ:
            return 1.0 / 416.0;
        case FREQ_833_HZ:
            return 1.0 / 833.0;
        case FREQ_1_66_KHZ:
            return 1.0 / 1660.0;
        case FREQ_3_33_KHZ:
            return 1.0 / 3330.0;
        case FREQ_6_66_KHZ:
            return 1.0 / 6660.0;
        case ACCEL_FREQ_1_6_HZ:
            return 1.0 / 1.6;
        default:
            return 0;
    }
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
void ISM330DHCX::getIMU() {
    uint32_t currentTime = timerGetTime();

    float gyroScale = gyroSensitivity * 0.001f;
    float accelScale = accelSensitivity;
    
    uint32_t delta = 0;

    if(currentTime >= lastTime)
        delta = currentTime - lastTime;
    else
        delta = (0xFFFFFFFF - lastTime + 1) + currentTime;

    float dt = delta / 1000000.0f;
    
    lastTime = currentTime;

    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    if(!readAccel(ax_raw, ay_raw, az_raw)) return;
    if(!readGyro(gx_raw, gy_raw, gz_raw)) return;

    float ax = ax_raw * accelScale;
    float ay = ay_raw * accelScale;
    float az = az_raw * accelScale;

    ax = accelFilterX.update(ax);
    ay = accelFilterY.update(ay);
    az = accelFilterZ.update(az);

    float gx = (float)gx_raw - gyroCalibrationX;
    float gy = (float)gy_raw - gyroCalibrationY;
    float gz = (float)gz_raw - gyroCalibrationZ;

    if (fabs(gx) < GYRO_NOISE_THRESHOLD) gx = 0.0f;
    if (fabs(gy) < GYRO_NOISE_THRESHOLD) gy = 0.0f;
    if (fabs(gz) < GYRO_NOISE_THRESHOLD) gz = 0.0f;

    gx *= gyroScale;
    gy *= gyroScale;
    gz *= gyroScale;

    FusionVector gyroscope = {gx, gy, gz};
    FusionVector accelerometer = {ax, ay, az};
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);
    quaternion = FusionAhrsGetQuaternion(&ahrs);
};



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