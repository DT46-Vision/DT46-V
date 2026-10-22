#pragma once
#include "modules/vision/pipeline.hpp"
namespace dt46 {
cv::Mat tracking_image(const cv::Mat& rgb,const Json& result);
cv::Mat ballistic_image(const cv::Mat& rgb,const Json& result);
cv::Mat observer_image(const cv::Mat& rgb,const Json& result,const cv::Matx44d& optical,const cv::Matx33d& k,const cv::Vec<double,5>& distortion);
}
