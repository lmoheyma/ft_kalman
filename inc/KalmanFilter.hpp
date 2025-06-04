#pragma once

#include <vector>

class KalmanFilter
{
    private:
        std::vector<double> _stateVector;
    public:
        KalmanFilter();
        ~KalmanFilter();
};
