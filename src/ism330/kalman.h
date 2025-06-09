class KalmanFilter1D{
public:
    KalmanFilter1D(float estimate, float errorCov, float processNoise, float measurementNoise);
    float update(float value);
private:
    float estimate;
    float errorCov;
    float processNoise;
    float measurementNoise;
};