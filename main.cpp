#include <iostream>
#include <cstdlib>
#include <time.h>
#include <unistd.h>
#include "KalmanFilter.hpp"

int main()
{
    KalmanFilter* KF = new KalmanFilter(4, 4, 2);
    KF->KalmanFilterInit();

    int cnt = 20;
    while(cnt > 0)
    {
        Eigen::MatrixXd u;
        u.resize(4, 1);
        u << 0.0, 0.0, 0.0, 0.0;

        srand((unsigned)time(NULL));
        double z1 = ((rand()/double(RAND_MAX)) - 0.5) / 10.0;       
        sleep(1);
        srand((unsigned)time(NULL));
        double z2 = ((rand()/double(RAND_MAX)) - 0.5) / 10.0;

        Eigen::MatrixXd z;
        z.resize(2, 1);
        z << z1, z2;

        KF->KalmanFilterRun(u, z);

        cnt--;
    }

    return 0;
}