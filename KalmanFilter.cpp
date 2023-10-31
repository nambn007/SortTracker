
#include <iostream>
#include <stdexcept>
#include "KalmanFilter.h"


KalmanFilter::KalmanFilter(int dim_x, int dim_z)
{
    this->dim_x = dim_x;
    this->dim_z = dim_z;
    
    this->x = Eigen::VectorXd::Zero(dim_x);
    this->P = Eigen::MatrixXd::Identity(dim_x, dim_x);
    this->Q = Eigen::MatrixXd::Identity(dim_x, dim_x);
    this->F = Eigen::MatrixXd::Identity(dim_x, dim_x);
    this->H = Eigen::MatrixXd::Zero(dim_z, dim_x);
    this->R = Eigen::MatrixXd::Identity(dim_z, dim_z);
    this->M = Eigen::MatrixXd::Zero(dim_z, dim_z);
    this->K = Eigen::MatrixXd::Zero(dim_x, dim_z);

    this->S = Eigen::MatrixXd::Zero(dim_z, dim_z);
    this->SI = Eigen::MatrixXd::Zero(dim_z, dim_z);   

    this->I = Eigen::MatrixXd::Identity(dim_x, dim_x);
}

KalmanFilter::KalmanFilter() 
{}

void KalmanFilter::predict()
{
    x = F * x;
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& z)
{
    Eigen::VectorXd y;
    Eigen::MatrixXd PHT;
    Eigen::MatrixXd I_KH;

    y = z - H * x;
    PHT = P * H.transpose();
    S = H * PHT + R;
    SI = S.inverse();
    K = PHT * SI;
    x = x + K * y;
    I_KH = I - K * H;
    P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
}