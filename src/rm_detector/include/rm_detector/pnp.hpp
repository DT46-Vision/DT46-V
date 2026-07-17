#ifndef PNP_H
#define PNP_H

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rm_detector/detector.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <cmath>
#include <vector>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;

namespace DT46_VISION {

    class PNP {
    public:
        // 构造函数，接受ROS 2日志器
        PNP(rclcpp::Logger logger) : logger_(logger) {}

        // 处理装甲板灯点坐标并返回 (dx, dy, dz)
        std::tuple<double, double, double, double> processArmorCorners(
        const sensor_msgs::msg::CameraInfo::SharedPtr& cam_info,
        bool use_geometric_center, // 是否用图像几何中心替代标定光心
        const cv::Mat& frame,
        const Armor& armor,
        int class_id
        );

    private:
        rclcpp::Logger logger_;
        cv::Mat cached_K_;
        cv::Mat cached_D_;
        bool has_cached_caminfo_ = false;

        void parseCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr& msg);
        std::vector<cv::Point3f> getObjectPoints(int object_size);
    };

}

#endif // PNP_H
