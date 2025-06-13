#include "../inc/KalmanFilter.hpp"

KalmanFilter::KalmanFilter() : I(identityMatrix(3))
{
}

KalmanFilter::~KalmanFilter()
{
}

Vector KalmanFilter::getStateVector(void) const
{
    return this->_stateVector;
}

void KalmanFilter::setStateVector(Vector stateVector)
{
    this->_stateVector = stateVector;
}

void KalmanFilter::initUncertaintyMatrix(void)
{
    this->R = matrixScalar(this->I, GPS_NOISE * GPS_NOISE);
}

Matrix KalmanFilter::propagationMatrix(void)
{
    Matrix upG = matrixScalar(identityMatrix(3), (DELTA_T * DELTA_T) * 1/2);
    Matrix lowG = matrixScalar(identityMatrix(3), DELTA_T);
    return mergeMatrixVertical(upG, lowG);
}

void KalmanFilter::initProcessNoiseMatrix(void)
{
    Matrix G = propagationMatrix();
    
}
