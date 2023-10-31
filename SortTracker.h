#pragma once

#include <vector>
#include "metadata.h"
#include "SortTrack.h"
#include "munkres/munkres.h"

void hungarian_matching(std::vector<std::vector<float>> iou_matrix, size_t nrows, size_t ncols, std::vector<std::vector<float>>& association)
{
    Matrix<float> matrix(nrows, ncols);
    for (int i = 0; i < nrows; i++) {
        for (int j = 0; j < ncols; j++) {
            if (iou_matrix[i][j] != 0) matrix(i, j) = -iou_matrix[i][j];
            else matrix(i, j) = 1.0f;
        }
    }

    Munkres<float> m;
    m.solve(matrix);

    for (size_t i = 0; i < nrows; i++) {
        for (size_t j = 0; j < ncols; j++) {
            association[i][j] = matrix(i, j);
        }
    }
}

void associate_detections_to_trackers(
    std::vector<Detection>& dets, 
    std::vector<SortTrack> tracks, 
    float iou_theshold=0.3)
{
    std::vector<std::pair<int, int>> matches;
    std::vector<int> unmatched_detections;
    std::vector<int> unmatched_tracks;

    if (tracks.empty()) {
        for (int i = 0; i < dets.size(); i++) {
            unmatched_detections.push_back(i);
        }
        return;
    }

    std::vector<std::vector<float>> iou_matrix;
    iou_matrix.resize(dets.size(), std::vector<float>(tracks.size()));
    
    for (int i = 0; i < dets.size(); i++) {
        for (int j = 0; j < tracks.size(); j++) {
            iou_matrix[i][j] = iou(dets[i].bbox, tracks[i].getStateDetection().bbox);
        }
    }

    std::vector<std::vector<float>> association;
    association.resize(dets.size(), std::vector<float>(tracks.size()));

    hungarian_matching(iou_matrix, dets.size(), tracks.size(), association);

    for (int i = 0; i < dets.size(); i++) {
        bool matched_flag = false;
        for (int j = 0; j < tracks.size(); j++) {
            if (association[i][j] == 0) {
                if (iou_matrix[i][j] > iou_theshold) {
                    matched_flag
                }
            }
        }
    }
}

class SortTracker
{
public:
    SortTracker(int max_age=1, int min_hits=3, float iou_threshold=0.3);
    ~SortTracker();

    void update(std::vector<Detection>& dets);

private:
    int max_age;
    int min_hits;
    int frame_count;
    float iou_threshold;
    std::vector<SortTrack> tracks; 
};


SortTracker::SortTracker(int max_age, int min_hits, float iou_threshold)
{
    this->max_age = max_age;
    this->min_hits = min_hits;
    this->iou_threshold = iou_threshold;
    this->tracks.clear();
    this->frame_count = 0;
}

SortTracker::~SortTracker() {}

void SortTracker::update(std::vector<Detection>& dets)
{
    this->frame_count++;

    std::remove_if(this->tracks.begin(), this->tracks.end(), [](SortTrack& track){
        Detection det = track.predict();
        if (det.bbox.x1 < 0 || det.bbox.y1 < 0 || det.bbox.x2 || det.bbox.y2 < 0)
            return true;
        return false;
    });

}