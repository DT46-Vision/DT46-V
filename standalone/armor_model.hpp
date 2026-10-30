#pragma once

#include <array>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

namespace dt46 {
struct ModelObservation {
    int armor_id=-1, robot_number=-1, team=-1;
    std::string sticker;
    bool vehicle=false;
    double presence=0, sticker_score=0, team_score=0;
    cv::Vec3d position{};
    cv::Matx33d rotation{};
    cv::Rect roi;
    std::array<cv::Point2f,4> corners{};
};

class ArmorModel {
public:
    explicit ArmorModel(const std::string& path, const cv::FileNode& parameters);
    std::vector<ModelObservation> detect(const cv::Mat& rgb, const cv::Matx33d& k,
                                         const cv::Vec<double,5>& d, int color);
    ModelObservation infer(const cv::Mat& crop, const cv::Matx33d& k, const cv::Vec<double,5>& d);
    static cv::Mat crop(const cv::Mat& rgb, const cv::Rect& roi, const cv::Matx33d& k, cv::Matx33d& crop_k);
    static cv::Matx33d decode_rotation(const float* axes);
    bool display=false;
    int light_count=0, candidate_count=0, invalid_pose_count=0;
    double candidate_ms=0, model_ms=0;
    // RGB previews: detections, search region, mask, model input, raw frame, gray.
    std::array<cv::Mat,6> images;
private:
    cv::dnn::Net net_;
    std::vector<std::string> classes_, families_;
    int binary_val_=120;
    double min_area_=5, min_ratio_=2, max_angle_=75, min_color_difference_=20;
    double height_ratio_=2.5, spacing_min_=.5, spacing_max_=4;
    double crop_height_=2.2, crop_margin_=1.35, presence_threshold_=.5;
    bool roi_crop_=false;
    double roi_scale_=.4;
};
}
