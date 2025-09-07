#ifndef _KALMAN_FILTER_1D_
#define _KALMAN_FILTER_1D_

class KalmanFilter1D
{
private:
    float _std; 
    float _r; // Variance
    float _x; 
    float _p;
    float _q;

    void updateVariables(){
        _r = _std * _std;
    }

public:
    KalmanFilter1D(float std, float Q, float x_init, float P_init);
    float update(float x);
};


#endif