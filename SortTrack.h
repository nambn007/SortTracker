#pragma once

#include <Eigen/Dense>
#include "metadata.h"
#include "KalmanFilter.h"

#define NUM_STATE 7
#define NUM_MESUAREMENTS 4

Eigen::VectorXd convertDet2VectorXd(Detection& det)
{
    Eigen::VectorXd x(NUM_STATE);
    float s = det.bbox.get_w() * det.bbox.get_h();
    float r = det.bbox.get_w() / det.bbox.get_h();
    x << det.bbox.xcenter(), det.bbox.ycenter(), s, r;
    return x;
}

Detection convertVectorXdtoDetection(Eigen::VectorXd& x)
{
    float w = std::sqrt(x[2] * x[3]);
    float h = x[2] / w;
    Detection det;
    det.bbox.x1 = x[0] - w / 2;
    det.bbox.y1 = x[1] - h / 2;
    det.bbox.x2 = x[0] + w / 2;
    det.bbox.y2 = x[1] + h / 2;
    return det;
} 

class SortTrack
{
public:
    SortTrack(Detection det);
    
    void update(Detection det);
    Detection predict();
    Detection getStateDetection();

private:
    int id;
    int age;
    int hits;
    int hit_streak;
    float time_since_update;
    std::list<Detection> det_history;
    KalmanFilter kf;
    
    int count = 0;
};

SortTrack::SortTrack(Detection det)
{
    int n = NUM_STATE; // Number of states
    int m = NUM_MESUAREMENTS; // Number of mesuarements

    this->kf = KalmanFilter(n, m);
    
    Eigen::MatrixXd F(n, n); // System dynamics matrix
    Eigen::MatrixXd H(m, n); // Output matrix 
    
    Eigen::MatrixXd Q(n, n); // Process noise convariance 
    Eigen::MatrixXd P(n, n); // Esitmate noise convariance
    Eigen::MatrixXd R(m, m); // Measurement noise convariance
    
    F << 1, 0, 0, 0, 1, 0, 0, 
         0, 1, 0, 0, 0, 1, 0,
         0, 0, 1, 0, 0, 0, 1,
         0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 1;
    
    H << 1, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0;

    R.block(2, 2, 2, 2) *= 10.0;
    P.block(4, 4, 3, 3) *= 1000;
    P *= 10.0f;
    Q(Q.rows() - 1, Q.cols() - 1) *= 0.01;
    Q.block(4, 4, 3, 3) *= 0.01;

    this->kf.x.head(4) = convertDet2VectorXd(det);
    
    this->time_since_update = 0;
    this->id = 0;
    this->det_history.clear();
    this->hits = 0;
    this->hit_streak = 0;
    this->age = 0;
}

void SortTrack::update(Detection det)
{
    this->time_since_update = 0;
    this->det_history.clear();
    this->hits += 1;
    this->hit_streak += 1;
    this->kf.update(convertDet2VectorXd(det));
}

Detection SortTrack::predict()
{
    if ((this->kf.x[6] + this->kf.x[2]) <= 0) {
        this->kf.x[6] *= 0.0f;
    }
    this->kf.predict();
    this->age += 1;
    if (this->time_since_update > 0) {
        this->hit_streak = 0;
    }
    this->time_since_update += 1;
    this->det_history.push_back(convertVectorXdtoDetection(this->kf.x));
    return this->det_history.back();
}

Detection SortTrack::getStateDetection()
{
    return convertVectorXdtoDetection(this->kf.x);
}
