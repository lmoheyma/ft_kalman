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
        Vector _acceleration;
        const Matrix I;
        Matrix P;
        Matrix Q;
        Matrix R;
        Matrix G;
        Matrix H;
        Matrix F;
        Matrix B;
        Matrix S;
    public:
        KalmanFilter();
        ~KalmanFilter();
        
        // Getter, Setter
        Vector getStateVector(void) const;
        void setStateVector(Vector stateVector);
        void setAcceleration(Vector acc);
        // Prediction, Update
        void predictStateVector(void);
        void update(void);
        // Matrix Initialization
        void initCovarianceMatrix();
        void initMeasurementMatrix(void);
        void initUncertaintyMatrix(void);
        void initStateTransitionMatrix(void);
        Matrix propagationMatrix(void);
        void initProcessNoiseMatrix(void);
        void initControlMatrix(void);
        void initObservationErrorCov(void);
};
