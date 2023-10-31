#pragma once

#include <Eigen/Dense>

/**
 * @brief Kalman filter
 * 
 */
class KalmanFilter 
{
public:
    KalmanFilter(int dim_x, int dim_y);
    KalmanFilter();
    
    /**
     * @brief Predict the estimated state
     * 
     */
    void predict();

    /**
     * @brief Update the estimated state
     * 
     * @param z: measurement
     */
    void update(const Eigen::VectorXd& z);

    int dim_x, dim_z;
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
};