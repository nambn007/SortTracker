#pragma once

#include <Eigen/Dense>

class KalmanFilter 
{
public:
    KalmanFilter(int dim_x, int dim_y);
    KalmanFilter();
    
    void predict();
    void update(const Eigen::VectorXd& z);

    Eigen::VectorXd x; // state
    Eigen::MatrixXd P; // uncertainty convariance
    Eigen::MatrixXd Q; // process uncertainty
    Eigen::MatrixXd F; // state transition matrix
    Eigen::MatrixXd H; // Measurement function
    Eigen::MatrixXd R; // state uncertainty
    Eigen::MatrixXd M; // process-measurement cross correlation
    Eigen::MatrixXd K; // kalman gain
    Eigen::MatrixXd S; // system uncertainty
    Eigen::MatrixXd SI; // inverse system uncertainty

    Eigen::MatrixXd I;
   
    int dim_x, dim_z;
};