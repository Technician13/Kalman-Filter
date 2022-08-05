#include <iostream>
#include "KalmanFilter.hpp"

/* constructor */
KalmanFilter::KalmanFilter(int dim_state_, int dim_control_, int dim_measure_):
    dim_state(dim_state_), dim_control(dim_control_), dim_measure(dim_measure_)
{
    x.resize(dim_state, 1);
    x_.resize(dim_state, 1);
    A.resize(dim_state, dim_state);
    B.resize(dim_state, dim_control);
    P.resize(dim_state, dim_state);
    P_.resize(dim_state, dim_state);
    Q.resize(dim_state, dim_state);
    y.resize(dim_measure, 1);
    z.resize(dim_measure, 1);
    H.resize(dim_measure, dim_state);
    R.resize(dim_measure, dim_measure);
    K.resize(dim_state, dim_measure);
    I = Eigen::MatrixXd::Identity(dim_state, dim_state);

    std::cout << "KalmanFilter Birth ..." << std::endl;
}

/* destructor */
KalmanFilter::~KalmanFilter()
{
    std::cout << "KalmanFilter Die ..." << std::endl;
}

/* init */
void KalmanFilter::KalmanFilterInit()
{
    /* ************************************ option start ************************************ */
    /* x init */
    x << 0.0, 0.0, 0.0, 0.0;
    #ifdef KALMANFILTER_TEST_PRINT_XINIT
        std::cout << "----------------------------------------- X INIT -----------------------------------------" << std::endl;
        PrintMatrix(x);
    #endif

    /* A init */
    A << 1.0, 0.0, 0.1, 0.0,
         0.0, 1.0, 0.0, 0.1,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    #ifdef KALMANFILTER_TEST_PRINT_A
        std::cout << "----------------------------------------- A -----------------------------------------" << std::endl;
        PrintMatrix(A);
    #endif

    /* B init */
    B << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    #ifdef KALMANFILTER_TEST_PRINT_B
        std::cout << "----------------------------------------- B -----------------------------------------" << std::endl;
        PrintMatrix(B);
    #endif

    /* P init */
    P << 1.0,   0.0,   0.0,   0.0,
         0.0,   1.0,   0.0,   0.0,
         0.0,   0.0,   100.0, 0.0,
         0.0,   0.0,   0.0,   100.0;
    #ifdef KALMANFILTER_TEST_PRINT_PINT
        std::cout << "----------------------------------------- P INIT -----------------------------------------" << std::endl;
        PrintMatrix(P);
    #endif

    /* Q init */
    Q << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    #ifdef KALMANFILTER_TEST_PRINT_Q
        std::cout << "----------------------------------------- Q -----------------------------------------" << std::endl;
        PrintMatrix(Q);
    #endif

    /* H init */
    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;
    #ifdef KALMANFILTER_TEST_PRINT_H
        std::cout << "----------------------------------------- H -----------------------------------------" << std::endl;
        PrintMatrix(H);
    #endif

    /* R init */
    R << 0.0225, 0.0,
         0.0,    0.0225;
    #ifdef KALMANFILTER_TEST_PRINT_R
        std::cout << "----------------------------------------- R -----------------------------------------" << std::endl;
        PrintMatrix(R);
    #endif
    /* ************************************ option end ************************************ */    
    std::cout << "KalmanFilter Init ..." << std::endl;
}

void KalmanFilter::KalmanFilterRun(Eigen::MatrixXd u)
{
    /* error checking */
    if(u.rows() != dim_control || u.cols() != 1)
        std::cout << "u is in wrong size !!!" << std::endl;
    
    u.resize(dim_control, 1);
    
    /* temp matrix S */
    Eigen::MatrixXd S;
    S.resize(dim_measure, dim_measure);

    x_ =  A * x + B * u;
    #ifdef KALMANFILTER_TEST_PRINT_X_
        std::cout << "----------------------------------------- X_ -----------------------------------------" << std::endl;
        PrintMatrix(x_);
    #endif

    P_ =  A * P * A.transpose() + Q;
    #ifdef KALMANFILTER_TEST_PRINT_P_
        std::cout << "----------------------------------------- P_ -----------------------------------------" << std::endl;
        PrintMatrix(P_);
    #endif

    y  =  z - H * x_;
    #ifdef KALMANFILTER_TEST_PRINT_Y
        std::cout << "----------------------------------------- Y -----------------------------------------" << std::endl;
        PrintMatrix(y);
    #endif

    S  =  H * P_ * H.transpose() + R; 
    #ifdef KALMANFILTER_TEST_PRINT_S
        std::cout << "----------------------------------------- S -----------------------------------------" << std::endl;
        PrintMatrix(S);
    #endif

    K  =  P_ * H.transpose() * S.inverse();
    #ifdef KALMANFILTER_TEST_PRINT_K
        std::cout << "----------------------------------------- K -----------------------------------------" << std::endl;
        PrintMatrix(K);
    #endif

    x  =  x_ + K * y;
    #ifdef KALMANFILTER_TEST_PRINT_X
        std::cout << "----------------------------------------- X -----------------------------------------" << std::endl;
        PrintMatrix(x);
    #endif

    P  =  (I - K * H) * P_;
    #ifdef KALMANFILTER_TEST_PRINT_P
        std::cout << "----------------------------------------- P -----------------------------------------" << std::endl;
        PrintMatrix(P);
    #endif
}

void PrintMatrix(Eigen::MatrixXd mat)
{
    for(int i = 0 ; i < mat.rows() ; i++)
    {
        for(int j = 0 ; j < mat.cols() ; j++)
        {
            std::cout << mat(i, j) << "  ";
        }
        std::cout << std::endl;
    }
}