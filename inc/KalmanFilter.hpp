#pragma once

#include <vector>
#include "../inc/maths.hpp"

#define ACCELEROMETER_NOISE 1e-3
#define GYROSCOPE_NOISE 1e-2
#define GPS_NOISE 1e-1

class KalmanFilter
{
    private:
        Vector _stateVector;
        const Matrix I;
        Matrix P;
        Matrix Q;
        Matrix R;
        Matrix G;
    public:
        KalmanFilter();
        ~KalmanFilter();
        Vector getStateVector(void) const;
        void setStateVector(Vector stateVector);
        void initUncertaintyMatrix(void);
        void initProcessNoiseMatrix(void);
};
