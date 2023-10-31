#pragma once

#include <vector>
#include "metadata.h"
#include "SortTrack.h"
#include "munkres/munkres.h"


namespace tracker 
{
/**
 * @brief Hungarian matching algorithm
 * 
 * @param iou_matrix: iou matrix from detections and tracked objects
 * @param nrows: number of detections
 * @param ncols: number of tracked objects
 * @param association: association matrix
 */
inline void hungarian_matching(std::vector<std::vector<float>> iou_matrix, size_t nrows, size_t ncols, std::vector<std::vector<float>>& association)
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

/**
    * @brief Associate detections to tracked object
    * @param dets: detections
    * @param tracks: tracked objects
    * @param iou_theshold: iou threshold
    * @param matches: matched pairs
    * @param unmatched_detections: unmatched detections
    * @param unmatched_tracks: unmatched tracks 
*/
inline void associate_detections_to_trackers(
    std::vector<Detection>& dets, 
    std::vector<SortTrack> tracks, 
    float iou_theshold,
    std::vector<std::pair<int,int>>& matches,
    std::vector<int>& unmatched_detections,
    std::vector<int>& unmatched_tracks)
{
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
        for (int j = 0; j < tracks.size(); j++) {
            if (association[i][j] == 0) {
                unmatched_detections.push_back(i);
                unmatched_tracks.push_back(j);
            }
            else if (association[i][j] < iou_theshold) {
                unmatched_detections.push_back(i);
                unmatched_tracks.push_back(j);
            }
            else {
                matches.push_back(std::make_pair(i, j));
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

}
