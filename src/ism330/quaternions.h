#pragma once

typedef struct{
    float roll;
    float pitch;
    float yaw;
} Euler;

class Quaternion{
public:
    Quaternion(float w, float x, float y, float z);
    void multiply(const Quaternion &q);
    void normalize();
    Euler toEuler();
    void slerp(Quaternion &q, float weight);
    void inverse();
    void ensurePositiveW();
    float w, x, y, z;
private:
   
};