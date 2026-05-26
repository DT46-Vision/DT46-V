#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <optional>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace dt46_vision {

class RmTF {
public:
    RmTF();

    /**
     * @brief 设置相机内参，并强制构建“理想光心” (图像几何中心)
     */
    void set_camera_info(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, int width = 640, int height = 480);

    /**
     * @brief 叠加并转换 IMU 欧拉角姿态，包含初始偏航角的相对归一化
     * @param raw_rpy 原始 [roll, pitch, yaw] (角度)
     * @param rotation_rpy 修正/外参 [roll, pitch, yaw] (角度)
     * @param order 旋转顺序 (目前仅支持 "xyz")
     */
    Eigen::Vector3d rotate_pose_axis(const Eigen::Vector3d& raw_rpy, const Eigen::Vector3d& rotation_rpy, const std::string& order = "xyz");

    /**
     * @brief 对三维位移向量应用旋转变换
     * @param raw_xyz 原始坐标 [x, y, z]
     * @param rotation_rpy 旋转补偿角 [roll, pitch, yaw] (角度)
     * @param order 旋转顺序 ("xyz" 或 "zyx")
     */
    Eigen::Vector3d rotate_pos_axis(const Eigen::Vector3d& raw_xyz, const Eigen::Vector3d& rotation_rpy, const std::string& order = "xyz");

    /**
     * @brief 将相机系下的 3D 坐标投影到 2D 像素平面
     * @param xyz_cam 相机系 3D 坐标
     * @return tuple<二维像素点, 是否在视野内>
     */

    // 新增函数的声明
    Eigen::Vector3d cam_to_world(const Eigen::Vector3d& raw_xyz, const Eigen::Vector3d& imu_rpy);
    Eigen::Vector3d world_to_cam(const Eigen::Vector3d& world_xyz, const Eigen::Vector3d& imu_rpy);

    std::tuple<cv::Point2i, bool> project_point(const Eigen::Vector3d& xyz_cam) const;

    bool has_camera_info() const { return has_camera_info_; }

private:
    bool has_camera_info_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    double cx_;
    double cy_;

    std::optional<double> initial_yaw_;

    // 内部高频数学工具：欧拉角转旋转矩阵
    Eigen::Matrix3d euler_to_matrix(const Eigen::Vector3d& rpy_deg, const std::string& order) const;
};

} // namespace dt46_vision
