#include "SortTracker.h"

namespace tracker 
{
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

    std::vector<std::pair<int, int>> matched;
    std::vector<int> unmatched_dets;
    std::vector<int> unmatched_tracks;
    associate_detections_to_trackers(dets, this->tracks, this->iou_threshold, matched, unmatched_dets, unmatched_tracks);

    // update matched trackers with assigned detections
    for (int i = 0; i < matched.size(); i++) {
        int track_idx = matched[i].first;
        int det_idx = matched[i].second;
        this->tracks[track_idx].update(dets[det_idx]);
        dets[det_idx].trackid = this->tracks[track_idx].id;
    }

    // create and initialize new trackers fo unmatched detections
    for (int i = 0; i < unmatched_dets.size(); i++) {
        int det_idx = unmatched_dets[i];
        SortTrack track(dets[det_idx]);
        this->tracks.push_back(track);
        dets[det_idx].trackid = track.id;
    }

    // remove all old tracks
    std::remove_if(this->tracks.begin(), this->tracks.end(), [this](SortTrack& track){
        if (track.time_since_update > this->max_age)
            return true;
        return false;
    });
}
}
