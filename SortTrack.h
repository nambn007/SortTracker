#pragma once

#include <Eigen/Dense>
#include "metadata.h"
#include "KalmanFilter.h"

namespace tracker
{
#define NUM_STATE 7
#define NUM_MESUAREMENTS 4

inline Eigen::VectorXd convertDet2VectorXd(Detection& det)
{
    Eigen::VectorXd x(NUM_STATE);
    float s = det.bbox.get_w() * det.bbox.get_h();
    float r = det.bbox.get_w() / det.bbox.get_h();
    x << det.bbox.xcenter(), det.bbox.ycenter(), s, r;
    return x;
}

inline Detection convertVectorXdtoDetection(Eigen::VectorXd& x)
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

    int id;
    int age;
    int hits;
    int hit_streak;
    float time_since_update;
    std::list<Detection> det_history;
    KalmanFilter kf;
    
    static int count;
};
}
