#pragma once

#include <vector>
#include "../inc/maths.hpp"

#define ACCELEROMETER_NOISE 1e-3
#define GYROSCOPE_NOISE 1e-2
#define GPS_NOISE 1e-1
#define DELTA_T 0.01

class KalmanFilter
{
    private:
        Vector _stateVector;
        const Matrix I;
        Matrix P;
        Matrix Q;
        Matrix R;
        Matrix G;
        Matrix H;
    public:
        KalmanFilter();
        ~KalmanFilter();
        Vector getStateVector(void) const;
        void setStateVector(Vector stateVector);
        void initCovarianceMatrix();
        void initMeasurementMatrix(void);
        void initUncertaintyMatrix(void);
        void initProcessNoiseMatrix(void);
        Matrix propagationMatrix(void);
};
