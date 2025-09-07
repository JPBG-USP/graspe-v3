#include <KalmanFilter1D.h>


/**
 * @brief A implementation of a 1D Kalman Filter, with only pos info
 * 
 * @param std Standar deviation of the sensor
 * @param Q Process noise variance
 * @param x_init Initial point prediction
 * @param P_init Initial Incertanty
 */
KalmanFilter1D::KalmanFilter1D(float std, float Q, float x_init, float P_init):
    _std(std), _q(Q), _x(x_init), _p(P_init){
        this->updateVariables();
    }

/**
 * @brief Update a Kalman Filter, and return the estimate position
 * 
 * @param measurement Measurement of the sensor
 */
float KalmanFilter1D::update(float measurement){
    // Prediction
    float x_pred = _x;
    float P_pred = _p + _q;

    // Correction
    float K = P_pred / (P_pred + _r);
    _x = x_pred + K * (measurement - x_pred);  
    _p = (1 - K) * P_pred;

    return _x;
}
