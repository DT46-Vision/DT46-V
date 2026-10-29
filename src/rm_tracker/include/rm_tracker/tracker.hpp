#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <optional>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "body_ekf.hpp"
#include "rm_tf_tools.hpp"

namespace dt46_vision {

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// 2. 装甲板数据结构
struct Armor {
    int id;
    Eigen::Vector3d pos; // x, y, z (世界系)
    double yaw;          // 绝对 Yaw 角
    int index;           // 板块序号 (0-3)
    double dist;         // 距离相机光心距离
    double angle_diff;   // 枪口偏离角度

    Armor(int id_, double x, double y, double z, double yaw_, int idx = -1)
        : id(id_), pos(x, y, z), yaw(yaw_), index(idx),
        dist(std::hypot(x, y, z)), angle_diff(0.0) {}
};

// 3. EKF 参数结构体
struct EKF_QR_Params {
    double q_xyz = 0.2;
    double q_yaw = 15.0;
    double q_r = 0.000001;
    double r_xyz_factor = 0.05;
    double r_depth_factor = 0.0; // Extra depth sigma = 5 mm + factor * .01 * range^2; 0 disables.
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
    BodyEkf::State body_state = BodyEkf::State::Zero();
    BodyEkf::Covariance body_covariance = BodyEkf::Covariance::Identity();
    bool body_ready = false;
    unsigned body_matched_mask = 0;
    int observed_plate = -1;
    double aim_lateral_sigma = 0, aim_vertical_sigma = 0, aim_hit_probability = 0;
    std::vector<std::string> fire_blockers;
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
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += 2.0 * M_PI;
    }
    return a - M_PI;
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
    // 额外执行延迟；选板与瞄准使用同一到达时刻。
    double controller_delay_ = 0.02;
    double bullet_speed = 28.0;
    double k_v2 = 0.019;
    double min_spinning_vel_ = 5.0;
    double spin_enter_time_ = 0.3;
    double spin_exit_time_ = 0.15;
    double shootable_dist = 3.0;
    double yaw_tolerance_deg = 5.0;
    double pitch_tolerance_deg = 2.0;
    double min_hit_probability = .5;

    TrackerState tracker_state;
    int tracked_id;


    // 【新增】：专门用于存放当前帧解算结果，供 UI 快照读取
    std::optional<Armor> target_cam_cache_ = std::nullopt;
    std::optional<Armor> target_muzzle_cache_ = std::nullopt;
    std::tuple<double, double, bool> gimbal_control_cache_ = {0.0, 0.0, false};

    // 【新增】：缓存当前帧所有原始装甲板观测，用于 UI 渲染
    std::vector<Armor> debug_yaw_armors_cache_;

    // 【新增】：声明快照提取函数
    RenderSnapshot get_render_snapshot() const;

    Eigen::Vector3d plate_position() const;
    Eigen::Vector3d plate_velocity() const;
    bool spinning() const { return spin_; }

    void build_ballistic_lut();

private:
    BodyEkf body_ekf_;
    bool body_spin_mode_=false;
    double body_spin_enter_s_=0,body_spin_exit_s_=0;
    double fire_yaw_tolerance_deg_=0,fire_pitch_tolerance_deg_=0;
    int body_plate_index_ = 0;
    bool body_matched_ = false;
    int selected_body_plate_ = -1;
    double detect_elapsed_s_ = 0, lost_elapsed_s_ = 0;
    double aim_lateral_sigma_ = 0, aim_vertical_sigma_ = 0;
    std::vector<std::string> fire_blockers_;
    Eigen::Matrix<double, 9, 1> target_state_;
    double dz_ = 0.0;
    double another_r_ = 0.0;
    bool spin_ = false;
    double aim_hit_probability_ = 0;

    // 弹道查找表
    static constexpr int LUT_DIST_BINS = 25;
    static constexpr int LUT_HEIGHT_BINS = 15;
    static constexpr double LUT_DIST_MIN = 0.1;
    static constexpr double LUT_DIST_MAX = 12.0;
    static constexpr double LUT_DIST_STEP = 0.5;
    static constexpr double LUT_HEIGHT_MIN = -3.5;
    static constexpr double LUT_HEIGHT_MAX = 3.5;
    static constexpr double LUT_HEIGHT_STEP = 0.5;
    double ballistic_lut_[LUT_DIST_BINS][LUT_HEIGHT_BINS];
    double tof_lut_[LUT_DIST_BINS][LUT_HEIGHT_BINS]; // 对应 (距离, 高度) 的子弹飞行时间 (s)

    inline double lut_lookup(double dist_h, double z) const;
    inline double lut_lookup_tof(double dist_h, double z) const;

    // =============== 内部逻辑函数 ===============
    void try_init_tracker(std::vector<Armor>& armors);
    void lock_target(const Armor& armor);
    void init_ekf(const Armor& armor);
    // FaterYU：装甲左右端点相对视线的角窗。delta_yaw 落在 (min,max) 内才允许打。
    bool yaw_in_armor_edge_window(double x, double y, double armor_yaw, double armor_w,
                                  double delta_yaw_rad, double shrink,
                                  double* allow_min_rad = nullptr,
                                  double* allow_max_rad = nullptr) const;

    std::optional<Armor> world_to_muzzle(RmTF& tf, const Armor& target, const Eigen::Vector3d& offset_pos, const Eigen::Vector3d& imu_rpy);


};

} // namespace dt46_vision
