#pragma once
#include <filesystem>
#include <opencv2/core.hpp>
#include <vector>

namespace dt46 {
struct Calibration {
    cv::Size image_size;
    cv::Mat k, distortion;
    double rms=0;
    std::vector<double> per_view_rms;
};
std::vector<cv::Point3f> chessboard_points(cv::Size corners,double square_metres);
Calibration calibrate(const std::vector<std::vector<cv::Point2f>>& observations,cv::Size image_size,cv::Size corners,double square_metres);
void save_calibration(const std::filesystem::path& path,const Calibration& calibration,const std::string& camera_name);
}
