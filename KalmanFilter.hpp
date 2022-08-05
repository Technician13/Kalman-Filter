#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

#include <Eigen/Dense>

/* ************************************ option start ************************************ */
/* print x init */
//#define KALMANFILTER_TEST_PRINT_XINIT
/* print A */
//#define KALMANFILTER_TEST_PRINT_A
/* print B */
//#define KALMANFILTER_TEST_PRINT_B
/* print P init */
//#define KALMANFILTER_TEST_PRINT_PINIT
/* print Q */
//#define KALMANFILTER_TEST_PRINT_Q
/* print H */
//#define KALMANFILTER_TEST_PRINT_H
/* print R */
//#define KALMANFILTER_TEST_PRINT_R
/* print x_ */
//#define KALMANFILTER_TEST_PRINT_X_
/* print P_ */
//#define KALMANFILTER_TEST_PRINT_P_
/* print y */
//#define KALMANFILTER_TEST_PRINT_Y
/* print S */
//#define KALMANFILTER_TEST_PRINT_S
/* print K */
//#define KALMANFILTER_TEST_PRINT_K
/* print x */
//#define KALMANFILTER_TEST_PRINT_X
/* print P */
//#define KALMANFILTER_TEST_PRINT_P
/* ************************************ option end ************************************ */

class KalmanFilter
{
    private:
        /* dim of state vector */
        int dim_state;
        /* dim of control vector */
        int dim_control;
        /* dim of measurement vector */
        int dim_measure;

        /* state matrix x & x_ */
        Eigen::MatrixXd x;
        Eigen::MatrixXd x_;
        /* transition matrix A */
        Eigen::MatrixXd A;
        /* control matrix B */
        Eigen::MatrixXd B;
        /* Cov matrix P & P_*/
        Eigen::MatrixXd P;
        Eigen::MatrixXd P_;
        /* process moise matrix Q */
        Eigen::MatrixXd Q;
        /* measurement error matrix y */
        Eigen::MatrixXd y;
        /* measurement result matrix z */
        Eigen::MatrixXd z;
        /* measurement matrix H */
        Eigen::MatrixXd H;
        /* measurement noise matrix R */
        Eigen::MatrixXd R;
        /* Kalman gain matrix K */
        Eigen::MatrixXd K;
        /* unit matrix */
        Eigen::MatrixXd I;

    protected:
        
    public:
        KalmanFilter(int dim_state_, int dim_control_, int dim_measure_);
        ~KalmanFilter();
        void KalmanFilterInit();
        void KalmanFilterRun(Eigen::MatrixXd u);
};

#endif