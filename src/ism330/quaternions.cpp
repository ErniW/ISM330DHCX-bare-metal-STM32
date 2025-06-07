#include "quaternions.h"
#include <math.h>

Quaternion::Quaternion(float w, float x, float y, float z){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
}

void Quaternion::multiply(const Quaternion &q){

    float w2 = w*q.w - x*q.x - y*q.y - z*q.z;
    float x2 = w*q.x + x*q.w + y*q.z - z*q.y;
    float y2 = w*q.y - x*q.z + y*q.w + z*q.x;
    float z2 = w*q.z + x*q.y - y*q.x + z*q.w;

    w = w2;
    x = x2;
    y = y2;
    z = z2;
}

void Quaternion::normalize(){
    float norm = sqrtf(w*w + x*x + y*y + z*z);
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
}

Euler Quaternion::toEuler(){
    Euler result = {0};

    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    result.roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

    float sinp = 2.0f * (w * y - z * x);
    if (fabs(sinp) >= 1)
        result.pitch = copysignf(90.0f, sinp);
    else
        result.pitch = asinf(sinp) * 180.0f / M_PI;

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    result.yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;

    return result;
}