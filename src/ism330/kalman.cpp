#include "kalman.h"

KalmanFilter1D::KalmanFilter1D(float estimate, float errorCov, float processNoise, float measurementNoise){
    this->estimate = estimate;
    this->errorCov = errorCov;
    this->processNoise = processNoise;
    this->measurementNoise = measurementNoise;
}

float KalmanFilter1D::update(float value){
    errorCov += processNoise;

    // Measurement update
    float kalmanGain = errorCov / (errorCov + measurementNoise);
    estimate += kalmanGain * (value - estimate);
    errorCov *= (1.0f - kalmanGain);

    return estimate;
}