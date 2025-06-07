#include "ism330.h"
#include "systick.h"
#include "timer.h"
#include "quaternions.h"

ISM330DHCX::ISM330DHCX(uint8_t address, I2C* i2c) : _address(address), _i2c(i2c), quaternion(1,0,0,0) {};

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
}

void ISM330DHCX::gyroInterruptEnable()
{
    _i2c->write(_address, COUNTER_BDR_REG1, DRDY_PULSE);
    _i2c->write(_address, INT1_CTRL, INT1_DRDY_G);
 
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

#define IMU_ALPHA 0.98
#define GYRO_NOISE_THRESHOLD 10

uint32_t lastTime = 0;

#define DEG2RAD (3.14159265359f / 180.0f)

void ISM330DHCX::getIMU(float& roll, float& pitch, float& yaw) {

    uint32_t currentTime = timerGetTime();

    float gyroScale = gyroSensitivity * 0.001f;
    float accelScale = accelSensitivity * 0.00980665f;
    
    // uint32_t currentTime = getMillis();
    // float dt = (currentTime - lastTime) / 1000.0f;
    // lastTime = currentTime;

    uint32_t delta = 0;

    if(currentTime >= lastTime)
        delta = currentTime - lastTime;
    else
        delta = (0xFFFFFFFF - lastTime + 1) + currentTime;

    float dt = delta / 1000000.0f;
    
    lastTime = currentTime;

    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    readAccel(ax_raw, ay_raw, az_raw);
    readGyro(gx_raw, gy_raw, gz_raw);

    float ax = ax_raw * accelScale;
    float ay = ay_raw * accelScale;
    float az = az_raw * accelScale;

    float gx = (float)gx_raw - gyroCalibrationX;
    float gy = (float)gy_raw - gyroCalibrationY;
    float gz = (float)gz_raw - gyroCalibrationZ;

    if (fabs(gx) < GYRO_NOISE_THRESHOLD) gx = 0.0f;
    if (fabs(gy) < GYRO_NOISE_THRESHOLD) gy = 0.0f;
    if (fabs(gz) < GYRO_NOISE_THRESHOLD) gz = 0.0f;

    gx *= gyroScale;
    gy *= gyroScale;
    gz *= gyroScale;

    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;

    float omega = sqrtf(gx*gx + gy*gy + gz*gz);
    if (omega > 0.0f) {
        float theta = omega * dt;
        float half_theta = 0.5f * theta;
        float sin_half_theta = sinf(half_theta);
        float ux = gx / omega;
        float uy = gy / omega;
        float uz = gz / omega;

        Quaternion dq(0,0,0,0);
        dq.w = cosf(half_theta);
        dq.x = ux * sin_half_theta;
        dq.y = uy * sin_half_theta;
        dq.z = uz * sin_half_theta;

        quaternion.multiply(dq);
        quaternion.normalize();
    }

    

    Quaternion accel(1,0,0,0);

    // Normalize accelerometer measurement
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 1e-6f) { // avoid div by zero
        ax /= norm; 
        ay /= norm; 
        az /= norm;

        // Reference gravity vector (down)
        const float refx = 0.0f;
        const float refy = 0.0f;
        const float refz = -1.0f;

        // Current gravity vector from quaternion (rotate vector [0,0,-1] by quaternion)
        float gx = 2.0f * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);
        float gy = 2.0f * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
        float gz = quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z;

        // Compute rotation axis (cross product between measured and expected gravity)
        float vx = ay * gz - az * gy;
        float vy = az * gx - ax * gz;
        float vz = ax * gy - ay * gx;

        // Compute angle between measured and expected gravity
        float dot = ax * gx + ay * gy + az * gz;

        if(dot > 1.0f) dot = 1.0f;
        if(dot < -1.0f) dot = -1.0f;

        float angle = acosf(dot);

        // Build correction quaternion representing this rotation
        float s = sinf(angle / 2.0f);
        Quaternion correction(cosf(angle / 2.0f), vx * s, vy * s, vz * s);
        correction.normalize();
         correction.ensurePositiveW();

 // Blend small correction into orientation quaternion
    // float correction_strength = 0.02f;  // tune this (small value)
    // quaternion.slerp(correction, correction_strength);
    // quaternion.ensurePositiveW();
    // quaternion.normalize();

        // Blend small correction into orientation quaternion
        float correction_strength = 0.02f;  // tune this (small value)
        
            Quaternion identity(1, 0, 0, 0);
        Quaternion blend = identity;
        blend.slerp(correction, correction_strength);

        blend.multiply(quaternion);  // blend = blend * quaternion
        quaternion = blend;
        quaternion.normalize(); 
        quaternion.ensurePositiveW();
    }

    

    
    // quaternion.slerp(accel, 0.02f);
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