#include "../inc/KalmanFilter.hpp"

KalmanFilter::KalmanFilter() : I(identityMatrix(3)) {}

KalmanFilter::~KalmanFilter() {}

Vector KalmanFilter::getStateVector(void) const {
    return this->_stateVector;
}

void KalmanFilter::setStateVector(Vector stateVector) {
    this->_stateVector = stateVector;
}

void KalmanFilter::setAcceleration(Vector acc) {
    this->_acceleration = acc;
}

void KalmanFilter::predictStateVector(void) {
    std::cout << "Acceleration :" << std::endl;
    printVector(this->_acceleration);
    Vector BuNoise = multiplyMatrixVector(this->B, this->_acceleration);
    Vector Fx = multiplyMatrixVector(this->F, this->_stateVector);
    try
    {
        this->_stateVector = addVectors(BuNoise, Fx);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void KalmanFilter::update(void) {
    return;
}

void KalmanFilter::initCovarianceMatrix(void) {
    Vector diagonal = {0.01, 0.01, 0.01, 0.5, 0.5, 0.5};
    this->P = diagonalMatrix(diagonal);
}

void KalmanFilter::initMeasurementMatrix(void) {
    this->H = mergeMatrixVertical(identityMatrix(3), Matrix(3, std::vector<double>(3, 0.0)));
    this->H = transpose(this->H);
}

void KalmanFilter::initUncertaintyMatrix(void) {
    this->R = matrixScalar(this->I, GPS_NOISE * GPS_NOISE);
}

void KalmanFilter::initStateTransitionMatrix(void) {
    size_t len = this->_stateVector.size();
    this->F = Matrix(len, Vector(len, 0.0));
    
    for (size_t i = 0; i < len; i++) {
        this->F[i][i] = 1.0;
    }
    for (size_t i = 0; i < (len/2); i++) {
        this->F[i][i + 3] = DELTA_T;
    }
}

Matrix KalmanFilter::propagationMatrix(void) {
    Matrix upG = matrixScalar(identityMatrix(3), (DELTA_T * DELTA_T) * 1/2);
    Matrix lowG = matrixScalar(identityMatrix(3), DELTA_T);
    return mergeMatrixVertical(upG, lowG);
}

void KalmanFilter::initProcessNoiseMatrix(void) {
    Matrix G = propagationMatrix();
    Matrix GTransposed = transpose(G);
 
    this->Q = multiply(G, GTransposed);
    this->Q = matrixScalar(this->Q, ACCELEROMETER_NOISE * ACCELEROMETER_NOISE);
}

void KalmanFilter::initControlMatrix(void) {
    Matrix upB = matrixScalar(identityMatrix(3), (DELTA_T * DELTA_T) * 1/2);
    Matrix lowB = matrixScalar(identityMatrix(3), DELTA_T);
    this->B = mergeMatrixVertical(upB, lowB);
}

void KalmanFilter::initObservationErrorCov(void) {
    this->S = multiply(this->H, this->P);
    this->S = multiply(this->S, transpose(this->H));
    this->S = addMatrix(this->S, this->R);
}
