#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <optional>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

// 提前声明 EKF 和 TF 工具类（我们之后会实现它们）
#include "ekf.hpp"
#include "rm_tf_tools.hpp"

namespace dt46_vision {

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// 1. 装甲板外观参数配置
struct RobotAppearance {
    int id;
    double armor_width;
    double armor_height;
    double armor_diagonal;
    double robot_r1;
    double robot_r2;

    RobotAppearance(int id_, double w, double h, double r1, double r2)
        : id(id_), armor_width(w), armor_height(h),
        armor_diagonal(std::hypot(w, h)), robot_r1(r1), robot_r2(r2) {}
};

// 2. 装甲板数据结构
struct Armor {
    int id;
    Eigen::Vector3d pos; // x, y, z (世界系)
    double yaw;          // 绝对 Yaw 角
    int index;           // 板块序号 (0-3)
    double dist;         // 距离相机光心距离
    double angle_diff;   // 枪口偏离角度
    double target_to_armor_dist;

    Armor(int id_, double x, double y, double z, double yaw_, int idx = -1)
        : id(id_), pos(x, y, z), yaw(yaw_), index(idx),
        dist(std::hypot(x, y, z)), angle_diff(0.0), target_to_armor_dist(0.0) {}
};

// 3. EKF 参数结构体
struct EKF_QR_Params {
    double q_xyz = 20.0;
    double q_yaw = 100.0;
    double q_r = 800.0;
    double r_xyz_factor = 0.05;
    double r_yaw = 0.02;
    double stable_dist = 1.5;
};

// 4. 半径限幅参数
struct RadiusParams {
    double r_max = 0.4;
    double r_min = 0.12;
};

// 状态机枚举
enum class TrackerState { LOST, DETECTING, TRACKING, TEMP_LOST };

// 渲染快照结构体
struct RenderSnapshot {
    TrackerState tracker_state;
    Eigen::Matrix<double, 9, 1> target_state;
    double another_r;
    double dz;
    std::vector<Armor> debug_yaw_armors;
    std::optional<Armor> target;
    std::optional<Armor> muzzle_target;
    bool spin;
    double yaw_tolerance_deg;
    double pitch_tolerance_deg;
    std::tuple<double, double, bool> gimbal_control;
    double ekf_yaw_vel;
    double bullet_speed;
};

// 角度工具函数
inline double normalize_angle(double angle) {
    return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}
inline double shortest_angular_distance(double from_rad, double to_rad) {
    return normalize_angle(to_rad - from_rad);
}

class Tracker {
public:
    Tracker();

    // =============== 核心业务接口 ===============
    // 主处理管线
    std::tuple<std::vector<double>, std::vector<std::pair<std::string, std::string>>>
    track(RmTF& tf, const std::vector<Armor>& raw_armors, const Eigen::Vector3d& imu_rpy, double dt);

    // 更新状态机与滤波器
    void update(const std::vector<Armor>& armors, double dt, std::vector<std::pair<std::string, std::string>>& logs);

    // 弹道求解
    std::tuple<double, double, bool> solve_ballistic(RmTF& tf, const Armor& muzzle_target,
                                                    const Eigen::Vector3d& cam_to_gun_rpy,
                                                    const Eigen::Vector3d& imu_rpy);

    // 判断是否允许开火
    std::tuple<double, double, bool> can_fire(const Armor& target, std::tuple<double, double, bool> gimbal_control);

    // =============== 系统参数 ===============
    int target_color = 0;
    Eigen::Vector3d cam_to_gun_pos = {0.0, -0.05, 0.0};
    Eigen::Vector3d cam_to_gun_rpy = {0.0, 0.0, 0.0};

    double max_match_distance = 0.2;
    double max_match_yaw_diff = 1.0; // 弧度
    int jump_cooldown_max = 20;
    int tracking_thres = 5;
    int lost_thres = 10;
    double dist_tol = 0.15;

    EKF_QR_Params ekf_QR_params;
    RadiusParams radius_params;

    double system_delay = 0.1;
    double bullet_speed = 28.0;
    double k_v2 = 0.019;
    double shootable_dist = 3.0;
    double yaw_tolerance_deg = 5.0;
    double pitch_tolerance_deg = 2.0;

    TrackerState tracker_state;
    int tracked_id;


    // 【新增】：专门用于存放当前帧解算结果，供 UI 快照读取
    std::optional<Armor> target_cam_cache_ = std::nullopt;
    std::optional<Armor> target_muzzle_cache_ = std::nullopt;
    std::tuple<double, double, bool> gimbal_control_cache_ = {0.0, 0.0, false};

    // 【新增】：声明快照提取函数
    RenderSnapshot get_render_snapshot() const;

private:
    ExtendedKalmanFilter ekf_;
    std::vector<RobotAppearance> robot_list_;

    int detect_count_ = 0;
    int lost_count_ = 0;
    int jump_cooldown_ = 0;

    Eigen::Matrix<double, 9, 1> target_state_;

    double last_yaw_ = 0.0;
    double dz_ = 0.0;
    double another_r_ = 0.23;
    bool spin_ = false;

    // 小陀螺状态机计数器
    int min_spinning_frame_count_ = 0;
    int spinning_frame_lost_count_ = 0;
    const int min_spinning_frame_ = 10;
    const int spinning_frame_lost_ = 5;
    const double min_spinning_vel_ = 5.0; // 进入小陀螺的最小角速度 (rad/s)

    // =============== 内部逻辑函数 ===============
    void try_init_tracker(std::vector<Armor>& armors);
    void lock_target(const Armor& armor);
    void init_ekf(const Armor& armor);
    void handle_armor_jump(const Armor& current_armor);

    Eigen::Matrix<double, 9, 1> predict_future_state() const;
    std::vector<Armor> find_all_armors(const Eigen::Matrix<double, 9, 1>& state) const;
    std::optional<Armor> find_target(std::vector<Armor>& robot_armors, const Eigen::Matrix<double, 9, 1>& state);
    std::optional<Armor> world_to_muzzle(RmTF& tf, const Armor& target, const Eigen::Vector3d& offset_pos, const Eigen::Vector3d& imu_rpy);

    Eigen::Vector3d get_armor_position_from_state(const Eigen::Matrix<double, 9, 1>& x) const;
    double orientation_to_yaw(double current_obs_yaw, double predicted_yaw) const;
};

} // namespace dt46_vision
