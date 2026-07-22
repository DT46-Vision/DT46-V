#include "rm_tracker/rm_tf_tools.hpp"

namespace dt46_vision {

RmTF::RmTF() : has_camera_info_(false), cx_(0.0), cy_(0.0), initial_yaw_(std::nullopt) {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
}

void RmTF::set_camera_info(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, int width, int height) {
    // 深拷贝确保内存安全，并强制转换为双精度浮点数供后续计算使用
    camera_matrix.convertTo(camera_matrix_, CV_64F);
    dist_coeffs.convertTo(dist_coeffs_, CV_64F);

    // ==========================================
    // 核心修改：光心矫正 (Virtual Ideal Camera)
    // ==========================================
    cx_ = width / 2.0;
    cy_ = height / 2.0;

    camera_matrix_.at<double>(0, 2) = cx_;
    camera_matrix_.at<double>(1, 2) = cy_;

    has_camera_info_ = true;

    // std::cout << "[RmTF] 虚拟理想相机构建完成: cx=" << cx_ << ", cy=" << cy_ << std::endl;
}

Eigen::Matrix3d RmTF::euler_to_matrix(const Eigen::Vector3d& rpy_deg, const std::string& order) const {
    double a0 = rpy_deg(0) * M_PI / 180.0;
    double a1 = rpy_deg(1) * M_PI / 180.0;
    double a2 = rpy_deg(2) * M_PI / 180.0;

    if (order == "xyz") {
        // Extrinsic XYZ: Rz(a2) * Ry(a1) * Rx(a0)
        return (Eigen::AngleAxisd(a2, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(a1, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(a0, Eigen::Vector3d::UnitX())).toRotationMatrix();
    }
    else if (order == "zyx") {
        // Extrinsic ZYX: Rx(a2) * Ry(a1) * Rz(a0)
        // a0 对应 Z 轴，a1 对应 Y 轴，a2 对应 X 轴
        return (Eigen::AngleAxisd(a2, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(a1, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(a0, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    }

    return Eigen::Matrix3d::Identity();
}

Eigen::Vector3d RmTF::rotate_pose_axis(const Eigen::Vector3d& raw_rpy, const Eigen::Vector3d& rotation_rpy, const std::string& order) {
    // 数据有效性校验防止 NaN 炸毁变换矩阵
    if (raw_rpy.hasNaN() || rotation_rpy.hasNaN()) {
        return raw_rpy;
    }

    // 1 & 2 & 3. 生成旋转矩阵并叠加 (等价于 scipy 的 R_final = r_current * r_fix)
    Eigen::Matrix3d R_current = euler_to_matrix(raw_rpy, order);
    Eigen::Matrix3d R_fix     = euler_to_matrix(rotation_rpy, order);
    Eigen::Matrix3d R_final   = R_current * R_fix;

    // 4. 将旋转矩阵还原为欧拉角 (严格解析 R_z * R_y * R_x 提取 roll, pitch, yaw)
    double pitch_rad = std::asin(std::clamp(-R_final(2, 0), -1.0, 1.0));
    double yaw_rad   = std::atan2(R_final(1, 0), R_final(0, 0));
    double roll_rad  = std::atan2(R_final(2, 1), R_final(2, 2));

    Eigen::Vector3d fixed_rpy(roll_rad * 180.0 / M_PI,
                              pitch_rad * 180.0 / M_PI,
                              yaw_rad * 180.0 / M_PI);

    // 记录开机后的第一帧 Yaw 为基准准星
    if (!initial_yaw_.has_value()) {
        initial_yaw_ = fixed_rpy(2);
    }

    // 计算相对初始姿态的角度
    double relative_yaw = fixed_rpy(2) - initial_yaw_.value();

    // 归一化到 [-180, 180] 之间
    double norm_yaw = std::fmod(relative_yaw + 180.0, 360.0);
    if (norm_yaw < 0) norm_yaw += 360.0;
    fixed_rpy(2) = norm_yaw - 180.0;

    return fixed_rpy;
}

Eigen::Vector3d RmTF::rotate_pos_axis(const Eigen::Vector3d& raw_xyz, const Eigen::Vector3d& rotation_rpy, const std::string& order) {
    // 1. 获取旋转关系矩阵
    Eigen::Matrix3d R_fix = euler_to_matrix(rotation_rpy, order);

    // 2 & 3. 矩阵乘法施加旋转
    return R_fix * raw_xyz;
}

//  修复 2: 补全从 Python 翻译过来的坐标系互转函数
Eigen::Vector3d RmTF::cam_to_world(const Eigen::Vector3d& raw_xyz, const Eigen::Vector3d& imu_rpy) {
    Eigen::Vector3d cam_fix(-90.0, 0.0, -90.0);
    Eigen::Vector3d normal_axis_xyz = rotate_pos_axis(raw_xyz, cam_fix, "xyz");
    return rotate_pos_axis(normal_axis_xyz, imu_rpy, "xyz");
}

Eigen::Vector3d RmTF::world_to_cam(const Eigen::Vector3d& world_xyz, const Eigen::Vector3d& imu_rpy) {
    Eigen::Vector3d inv_imu_rpy(-imu_rpy(2), -imu_rpy(1), -imu_rpy(0));
    Eigen::Vector3d normal_axis_xyz = rotate_pos_axis(world_xyz, inv_imu_rpy, "zyx");
    Eigen::Vector3d cam_inv_fix(90.0, 0.0, 90.0);
    return rotate_pos_axis(normal_axis_xyz, cam_inv_fix, "zyx");
}

std::tuple<cv::Point2i, bool> RmTF::project_point(const Eigen::Vector3d& xyz_cam) const {
    if (!has_camera_info_) {
        return {cv::Point2i(0, 0), false};
    }

    double Z = xyz_cam(2);
    if (std::abs(Z) < 1e-6) {
        return {cv::Point2i(0, 0), false};
    }

    double fx = camera_matrix_.at<double>(0, 0);
    double fy = camera_matrix_.at<double>(1, 1);
    double cx = camera_matrix_.at<double>(0, 2);
    double cy = camera_matrix_.at<double>(1, 2);

    int u = static_cast<int>(fx * xyz_cam(0) / Z + cx + 0.5);
    int v = static_cast<int>(fy * xyz_cam(1) / Z + cy + 0.5);

    bool is_valid = (u > -10000 && u < 10000 && v > -10000 && v < 10000);
    return {cv::Point2i(u, v), is_valid};
}

} // namespace dt46_vision
