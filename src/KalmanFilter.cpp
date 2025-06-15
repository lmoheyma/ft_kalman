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

void KalmanFilter::initCovarianceMatrix(void)
{
    Vector diagonal = {0.01, 0.01, 0.01, 0.5, 0.5, 0.5};
    this->P = diagonalMatrix(diagonal);
    
    std::cout << "P : " << std::endl;
    printMatrix(this->P);
}

void KalmanFilter::initMeasurementMatrix(void)
{
    this->H = mergeMatrixVertical(identityMatrix(3), Matrix(3, std::vector<double>(3, 0.0)));
    this->H = transpose(this->H);
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
    Matrix GTransposed = transpose(G);
 
    this->Q = multiply(G, GTransposed);
    this->Q = matrixScalar(this->Q, ACCELEROMETER_NOISE * ACCELEROMETER_NOISE);
}
