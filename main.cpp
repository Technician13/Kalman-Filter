#include <iostream>
#include "KalmanFilter.hpp"

int main()
{
    KalmanFilter* KF = new KalmanFilter(4, 4, 2);
    KF->KalmanFilterInit();

    Eigen::MatrixXd u;
    u.resize(4, 1);
    u << 0.0, 0.0, 0.0, 0.0;

    KF->KalmanFilterRun(u);

    return 0;
}


















