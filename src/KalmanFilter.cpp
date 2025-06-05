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

void KalmanFilter::initProcessNoiseMatrix(void)
{
    double deltaT = 0.01;

    Matrix upG = matrixScalar(identityMatrix(3), (1/2) * (deltaT * deltaT));
    Matrix lowG = matrixScalar(identityMatrix(3), deltaT);
    Matrix G = mergeMatrixVertical(upG, lowG);
    printMatrix(G);
}
