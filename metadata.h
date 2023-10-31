#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>

using time_point = std::chrono::system_clock::time_point;

template<typename T>
inline T get_current_time() {
return std::chrono::duration_cast<std::chrono::duration<T>>(std::chrono::system_clock::now().time_since_epoch()).count();    
}


struct BBox {
    float x1;
    float y1;
    float x2;
    float y2;
    
    float get_w() {
        return x2 - x1;
    }
    float get_h() {
        return y2 - y1;
    }
    float xcenter() {
        return (x1 + x2) / 2; 
    }
    float ycenter() {
        return (y1 + y2) / 2;
    }
    int get_w_int() {
        return (int) (x2 - x1);
    }
    int get_h_int() {
        return (int) (y2 - y1);
    }
    int get_x1_int() {
        return (int) x1;
    }
    int get_y1_int() {
        return (int) y1;
    }
    int get_x2_int() {
        return (int) x2;
    }
    int get_y2_int() {
        return (int) y2;
    }
    void fromxywh(float cx, float cy, float w, float h) {
        x1 = cx - w / 2;
        y1 = cy - h / 2;
        x2 = cx + w / 2;
        y2 = cy + h / 2;
    }
    cv::Rect2f toRect2f() {
        return cv::Rect2f(x1, y1, get_w(), get_h());
    }
    void set_wh(int w, int h) {
        x1 = x1 * w * 1.0f;
        x2 = x2 * w * 1.0f;
        y1 = y1 * h * 1.0f;
        y2 = y2 * h * 1.0f;
    }
    bool is_absolute() {
        std::vector<float> temp = {x1, x2, y1, y2};
        bool is_abs = false;
        for (auto e : temp) {
            if (e > 1.0f) is_abs = true;
        }
        return is_abs;
    }
    void open_box(float open_w, float open_h, float max_w, float max_h) {
        float w_open = get_w() * open_w;
        float h_open = get_h() * open_h;
        x1 = std::max(0.0f, x1 - w_open / 2);
        y1 = std::max(0.0f, y1 - h_open / 2);
        x2 = std::min(max_w, x2 + w_open / 2);
        y2 = std::min(max_h, y2 + h_open / 2);
    }
    void print_info() {
        printf("xywh: %.1f-%.1f-%.1f-%.1f\n", x1, y1, get_w(), get_h());
    }
};

struct Landmark {
    float points[10];
};

enum ObjectClass
{
    FACE,
    HEAD,
    HUMAN,
    LICENSE_PLATE
};

struct Detection 
{
    int trackid;
    int labelid;
    float label_prob;
    float prob;
    BBox bbox;
    ObjectClass oclass;
    Landmark lmk;
};


inline float calc_intersection_area(BBox& box1, BBox& box2) {
  float intersection_w = std::min(box1.x2, box2.x2) - std::max(box1.x1, box2.x1);
  intersection_w = std::max(0.0f, intersection_w);
  float intersection_h = std::min(box1.y2, box2.y2) - std::max(box1.y1, box2.y1);
  intersection_h = std::max(0.0f, intersection_h);
  float intersection_area = std::max(intersection_h * intersection_w, 0.0f);
  return intersection_area;
}

inline float iou(BBox box1, BBox box2) {
  float w1 = box1.get_w();
  float h1 = box1.get_h();
  float w2 = box2.get_w();
  float h2 = box2.get_h();
    
  if (w1 <= 0 && h1 <= 0 && w2 <= 0 && h2 <= 0) {
    return 0;
  }

  float intersection_area = calc_intersection_area(box1, box2);
  float area1 = w1 * h1;
  float area2 = w2 * h2;
  float union_area = area1 + area2 - intersection_area;

  return intersection_area / union_area;
}