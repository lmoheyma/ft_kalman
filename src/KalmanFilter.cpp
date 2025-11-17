#include "../inc/KalmanFilter.hpp"

KalmanFilter::KalmanFilter() : I(identityMatrix(3)) {}

KalmanFilter::~KalmanFilter() {}

Vector KalmanFilter::getStateVector(void) const {
    return this->_stateVector;
}

void KalmanFilter::setStateVector(Vector stateVector) {
    this->_stateVector = stateVector;
}

Vector KalmanFilter::getAcceleration(void) const {
    return this->_acceleration;
}

void KalmanFilter::setAcceleration(Vector acc) {
    this->_acceleration = acc;
}

Matrix KalmanFilter::getP(void) const {
    return this->P;
}

void KalmanFilter::predictStateVector(void) {
    Vector BuNoise = multiplyMatrixVector(this->B, this->_acceleration);
    Vector Fx = multiplyMatrixVector(this->F, this->_stateVector);
    try
    {
        this->_stateVector = addVectors(BuNoise, Fx);
        this->P = addMatrix(multiply(multiply(this->F, this->P), transpose(this->F)), this->Q);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void KalmanFilter::update(const Vector& gps_measurement) {
    // Innovation (erreur d'observation)
    Vector Hx = multiplyMatrixVector(this->H, this->_stateVector); // H * x
    Vector y(gps_measurement.size());
    for (size_t i = 0; i < gps_measurement.size(); ++i)
        y[i] = gps_measurement[i] - Hx[i];

    // Matrice d'innovation S = HPH^T + R
    Matrix HP = multiply(this->H, this->P);
    Matrix HPHt = multiply(HP, transpose(this->H));
    Matrix S = addMatrix(HPHt, this->R);
    Matrix PHt = multiply(this->P, transpose(this->H));
    Matrix S_inv = inverseMatrix(S);
    Matrix K = multiply(PHt, S_inv);

    // Mise à jour de l'état x = x + K * y
    Vector Ky(K.size(), 0.0);
    for (size_t i = 0; i < K.size(); ++i)
        for (size_t j = 0; j < y.size(); ++j)
            Ky[i] += K[i][j] * y[j];
    for (size_t i = 0; i < this->_stateVector.size(); ++i)
        this->_stateVector[i] += Ky[i];

    // Mise à jour de la covariance P = (I - K*H) * P
    Matrix KH = multiply(K, this->H);
    Matrix I = identityMatrix(this->P.size());
    Matrix I_KH = I;
    for (size_t i = 0; i < I.size(); ++i)
        for (size_t j = 0; j < I[0].size(); ++j)
            I_KH[i][j] = I[i][j] - KH[i][j];
    this->P = multiply(I_KH, this->P);
}

void KalmanFilter::initCovarianceMatrix(void) {
    double pos_var = GPS_NOISE * GPS_NOISE;
    double vel_var = GYROSCOPE_NOISE * GYROSCOPE_NOISE + ACCELEROMETER_NOISE * ACCELEROMETER_NOISE * DELTA_T;

    Vector diagonal = {pos_var, pos_var, pos_var, vel_var, vel_var, vel_var};
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
    Matrix upG = matrixScalar(identityMatrix(3), (DELTA_T * DELTA_T) * 0.5);
    Matrix lowG = matrixScalar(identityMatrix(3), DELTA_T);
    return mergeMatrixVertical(upG, lowG);
}

void KalmanFilter::initProcessNoiseMatrix(void) {
    Matrix G = propagationMatrix();
    Matrix GTransposed = transpose(G);
 
    double acc_variance = ACCELEROMETER_NOISE * ACCELEROMETER_NOISE;
    double gyro_variance = GYROSCOPE_NOISE * GYROSCOPE_NOISE;

    double total_acc_variance = acc_variance + gyro_variance;

    this->Q = multiply(G, GTransposed);
    this->Q = matrixScalar(this->Q, total_acc_variance);
}

void KalmanFilter::initControlMatrix(void) {
    Matrix upB = matrixScalar(identityMatrix(3), (DELTA_T * DELTA_T) * 0.5);
    Matrix lowB = matrixScalar(identityMatrix(3), DELTA_T);
    this->B = mergeMatrixVertical(upB, lowB);
}
