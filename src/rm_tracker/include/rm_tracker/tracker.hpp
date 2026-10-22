#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <deque>
#include <optional>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

// 提前声明 EKF 和 TF 工具类
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
    double q_xyz = 2.5;
    double q_yaw = 15.0;
    double q_r = 0.05;
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
    // 云台多出来的跟踪滞后；只加在瞄准点外推，不影响选板/正对判定（FYT controller_delay）。
    double controller_delay_ = 0.02;
    double bullet_speed = 28.0;
    double k_v2 = 0.019;
    double gravity = 9.81;
    double min_spinning_vel_ = 5.0;
    double spin_enter_time_ = 0.3;
    double spin_exit_time_ = 0.15;
    double shootable_dist = 3.0;
    double yaw_tolerance_deg = 5.0;
    double pitch_tolerance_deg = 2.0;

    TrackerState tracker_state;
    int tracked_id;

    bool aim_valid = false;

    // 【新增】：专门用于存放当前帧解算结果，供 UI 快照读取
    std::optional<Armor> target_cam_cache_ = std::nullopt;
    std::optional<Armor> target_muzzle_cache_ = std::nullopt;
    std::tuple<double, double, bool> gimbal_control_cache_ = {0.0, 0.0, false};

    // 【新增】：缓存当前帧所有原始装甲板观测，用于 UI 渲染
    std::vector<Armor> debug_yaw_armors_cache_;

    // 【新增】：声明快照提取函数
    RenderSnapshot get_render_snapshot() const;

    Eigen::Vector3d plate_position() const {
        return Eigen::Vector3d(plate_(0), plate_(2), plate_(4));
    }
    Eigen::Vector3d plate_velocity() const {
        return Eigen::Vector3d(plate_(1), plate_(3), plate_(5));
    }
    bool spinning() const { return spin_; }
    const std::optional<Armor>& plate_measurement() const { return plate_meas_; }

    void build_ballistic_lut();

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

    // 击打用的是这块板自己的位置和速度，不把板的位移写进车心。
    // 状态 [x, vx, y, vy, z, vz]，世界系。
    Eigen::Matrix<double, 6, 1> plate_ = Eigen::Matrix<double, 6, 1>::Zero();
    // 提前量用约 0.15s 的水平位移。0.4s 平均跟不上速度变化，飞到时会差出一块小装甲。
    Eigen::Vector3d lead_vel_ = Eigen::Vector3d::Zero();
    bool plate_inited_ = false;
    std::optional<Armor> plate_meas_;
    double last_plate_yaw_ = 0.0;
    int ekf_reject_streak_ = 0;

    // 换板锁存：切向跳变同号连续两次置上，4s 没有再跳、板也不再快移才清。
    // 正弦慢半圈里跳变间隔会超过 2s，短于这段会把标志清掉，然后按绕心速度提前。
    int spin_jump_count_ = 0;
    int last_jump_sign_ = 0;
    double spin_quiet_ = 0.0;
    double plate_move_speed_ = 0.0;
    struct MoveSample {
        double t;
        Eigen::Vector3d pos;
    };
    std::deque<MoveSample> move_hist_;
    struct XySample {
        double t;
        double x;
        double y;
    };
    // 仅双板解出的车心，用来判平移。
    std::deque<XySample> obs_center_hist_;
    std::deque<XySample> plate_arc_;
    bool spin_center_valid_ = false;
    double spin_center_x_ = 0.0;
    double spin_center_y_ = 0.0;
    // bag 扫参用：最近一帧双板解心相对车心的距离；无双板为 -1。
    double last_dual_innov_ = -1.0;
    std::deque<XySample> fit_hist_;
    // 同向换板间隔。相邻板差 90°，转速 = (π/2) / 间隔。
    std::deque<double> spin_periods_;
    double last_same_jump_clock_ = -1.0;
    double move_clock_ = 0.0;
    double last_jump_clock_ = -1.0;
    // 最近一次由换板周期算出的 v_yaw。周期窗暂时空时沿用。
    double last_spin_vyaw_ = 0.0;
    // SPIN 瞄法：false=跟板等正对；true=车心正前（平移小陀螺）。滞回防撕枪。
    bool spin_aim_translate_ = false;
    double spin_aim_mode_clock_ = 0.0;
    int spin_front_plate_i_ = 0;
    static constexpr double kMoveWindow = 0.40;
    static constexpr double kSpinQuiet = 4.0;
    static constexpr double kJumpTangential = 0.12;
    static constexpr double kSameRobot = 0.8;
    static constexpr double kSwitchThrRad = 15.0 * M_PI / 180.0;
    static constexpr double kSwitchAdvMax = 0.2;
    static constexpr double kLeaveBiasRad = 50.0 * M_PI / 180.0;
    static constexpr double kAimEnterTrans = 0.45; // 判平移后多久切正前（偏慢，防原地误切）
    static constexpr double kAimLeaveTrans = 0.30; // 判静止后多久切回跟板（偏快收回）

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
    void handle_armor_jump(const Armor& current_armor);
    void reset_plate(const Armor& armor);
    void update_plate(const std::vector<Armor>& armors, double dt,
                      std::vector<std::pair<std::string, std::string>>& logs);
    void update_lead_velocity(double dt);
    void note_plate_motion(const Eigen::Vector3d& pos, double dt, bool jumped);
    void note_plate_jump(const Eigen::Vector3d& pred, const Eigen::Vector3d& meas,
                         std::vector<std::pair<std::string, std::string>>& logs);
    void spin_quiet_tick(double dt, bool jumped,
                         std::vector<std::pair<std::string, std::string>>& logs);
    bool apply_armor_measurement(const Armor& armor);
    void note_plate_arc(double x, double y);
    bool center_motion_is_slow() const;
    void hold_center_when_stationary(const std::vector<Armor>& armors);
    void apply_spin_period();
    void update_spin_aim_mode(double dt);
    // 单板：板位 + r 解车心（yaw 二义消解）。失败返回 false。
    bool center_from_one_armor(const Armor& armor, double r, double& xc, double& yc) const;
    // 平移 SPIN：到达时刻选正对板（滞回 + 提前量）。
    std::pair<int, double> select_spin_front_plate(double front_yaw, double yaw_arr, double v_yaw);
    // FaterYU：装甲左右端点相对视线的角窗。delta_yaw 落在 (min,max) 内才允许打。
    bool yaw_in_armor_edge_window(double x, double y, double armor_yaw, double armor_w,
                                  double delta_yaw_rad, double shrink,
                                  double* allow_min_rad = nullptr,
                                  double* allow_max_rad = nullptr) const;

    std::optional<Armor> world_to_muzzle(RmTF& tf, const Armor& target, const Eigen::Vector3d& offset_pos, const Eigen::Vector3d& imu_rpy);

    Eigen::Vector3d get_armor_position_from_state(const Eigen::Matrix<double, 9, 1>& x) const;
    double orientation_to_yaw(double current_obs_yaw, double predicted_yaw) const;

    // PnP 板 yaw 的镜像二义用位置消歧（见 tracker.cpp 实现处注释）
    double disambiguate_armor_yaw(double obs_yaw, const Eigen::Vector3d& armor_pos,
                                    double ref_yaw, double ref_x, double ref_y) const;

    // 在 armor±r·u(yaw) 两个候选里定车心，避免自旋时心跑到板外侧
    void resolve_center_from_armor(double xa, double ya, double r, double& yaw,
                                    double& xc, double& yc,
                                    bool has_prev, double prev_xc, double prev_yc) const;
};

} // namespace dt46_vision
