#include "rm_tracker/tracker.hpp"
#include <algorithm>

namespace dt46_vision {

Tracker::Tracker() {
    tracker_state = TrackerState::LOST;
    tracked_id = -1;
    target_state_.setZero();

    // 初始化机器人列表 (部分示例，对应 Python 的 robot_list)
    robot_list_.emplace_back(0, 230.0, 125.0, 0.3, 0.285);
    robot_list_.emplace_back(1, 135.0, 125.0, 0.23, 0.21);
    robot_list_.emplace_back(2, 135.0, 125.0, 0.3, 0.27);
    robot_list_.emplace_back(3, 135.0, 125.0, 0.23, 0.21);
    robot_list_.emplace_back(4, 135.0, 125.0, 0.23, 0.21);
    robot_list_.emplace_back(5, 135.0, 125.0, 0.24, 0.22);
    robot_list_.emplace_back(6, 230.0, 125.0, 0.3, 0.285);
    robot_list_.emplace_back(7, 135.0, 125.0, 0.23, 0.21);
    // 可根据实际情况补全
}

Eigen::Vector3d Tracker::get_armor_position_from_state(const Eigen::Matrix<double, 9, 1>& x) const {
    double xc = x(0), yc = x(2), za = x(4);
    double yaw = x(6), r = x(8);
    return Eigen::Vector3d(xc - r * std::cos(yaw), yc - r * std::sin(yaw), za);
}

double Tracker::orientation_to_yaw(double current_obs_yaw, double predicted_yaw) const {
    double diff = shortest_angular_distance(predicted_yaw, current_obs_yaw);
    return predicted_yaw + diff;
}

void Tracker::init_ekf(const Armor& armor) {
    double xa = armor.pos(0), ya = armor.pos(1), za = armor.pos(2);
    last_yaw_ = armor.yaw;

    double r_init = 0.26; // 默认防错半径
    for(const auto& r : robot_list_) {
        if(r.id == armor.id) {
            r_init = r.robot_r1;
            another_r_ = r.robot_r2;
            break;
        }
    }

    // 假定你的 ekf_.init() 会初始化 9 维状态
    Eigen::Matrix<double, 9, 1> init_x;
    init_x << xa + r_init * std::cos(last_yaw_), 0,
            ya + r_init * std::sin(last_yaw_), 0,
            za, 0, last_yaw_, 0, r_init;

    Eigen::Matrix<double, 9, 9> init_p = Eigen::Matrix<double, 9, 9>::Identity();
    init_p(1,1) = 50.0; init_p(3,3) = 50.0; init_p(5,5) = 10.0; init_p(7,7) = 50.0;

    ekf_.init(init_x, init_p);
    target_state_ = init_x;
    dz_ = 0.0;
}

void Tracker::handle_armor_jump(const Armor& current_armor) {
    double yaw = orientation_to_yaw(current_armor.yaw, target_state_(6));
    target_state_(6) = yaw;

    // 高度钳制
    double raw_dz = target_state_(4) - current_armor.pos(2);
    dz_ = std::clamp(raw_dz, -0.085, 0.085);
    target_state_(4) = current_armor.pos(2);
    target_state_(5) = 0.0; // 斩断 Z 轴错误速度积分

    // 水平速度衰减
    target_state_(1) *= 0.8;
    target_state_(3) *= 0.8;

    // 半径交换
    std::swap(target_state_(8), another_r_);

    // 中心位置强制校正
    double r = target_state_(8);
    target_state_(0) = current_armor.pos(0) + r * std::cos(yaw);
    target_state_(2) = current_armor.pos(1) + r * std::sin(yaw);

    // 重新写回 EKF
    ekf_.setState(target_state_);
    // 注意：需在 EKF 中实现 smooth_reset_covariance 方法
    ekf_.smooth_reset_covariance();
}

void Tracker::update(const std::vector<Armor>& armors, double dt) {
    if (jump_cooldown_ > 0) jump_cooldown_--;

    ekf_.predict(dt);
    target_state_ = ekf_.getState();

    bool matched = false;
    std::optional<Armor> best_match = std::nullopt;
    double min_position_diff = std::numeric_limits<double>::infinity();
    double yaw_diff = std::numeric_limits<double>::infinity();

    Eigen::Vector3d pred_armor_pos = get_armor_position_from_state(target_state_);
    Eigen::Vector3d pred_center_pos(target_state_(0), target_state_(2), target_state_(4));

    if (!armors.empty()) {
        for (const auto& armor : armors) {
            if (armor.id == tracked_id) {
                double p_diff = (pred_armor_pos - armor.pos).norm();
                if (p_diff < min_position_diff) {
                    min_position_diff = p_diff;
                    yaw_diff = std::abs(shortest_angular_distance(target_state_(6), armor.yaw));
                    best_match = armor;
                }
            }
        }

        if (best_match.has_value()) {
            if (min_position_diff < max_match_distance && yaw_diff < max_match_yaw_diff) {
                // 完美匹配
                matched = true;
                double cont_yaw = orientation_to_yaw(best_match->yaw, target_state_(6));
                Eigen::Vector4d measurement(best_match->pos(0), best_match->pos(1), best_match->pos(2), cont_yaw);
                ekf_.update(measurement); // 假设 ekf 接收 4 维观测
                target_state_ = ekf_.getState();
            } else if (yaw_diff > max_match_yaw_diff) {
                // 装甲板跳变
                if (jump_cooldown_ == 0 && (pred_center_pos - best_match->pos).norm() < 0.6) {
                    handle_armor_jump(*best_match);
                    jump_cooldown_ = jump_cooldown_max;
                    matched = true;
                }
            }
        }
    }

    // 钳制状态范围防发散
    target_state_(8) = std::clamp(target_state_(8), radius_params.r_min, radius_params.r_max);
    target_state_(1) = std::clamp(target_state_(1), -15.0, 15.0);
    target_state_(3) = std::clamp(target_state_(3), -15.0, 15.0);
    target_state_(5) = std::clamp(target_state_(5), -2.0, 2.0);
    ekf_.setState(target_state_);

    // 状态机流转
    if (tracker_state == TrackerState::DETECTING) {
        if (matched) {
            if (++detect_count_ > tracking_thres) {
                tracker_state = TrackerState::TRACKING;
                detect_count_ = 0;
            }
        } else {
            if (--detect_count_ <= 0) {
                tracker_state = TrackerState::LOST;
            }
        }
    } else if (tracker_state == TrackerState::TRACKING) {
        if (!matched) {
            tracker_state = TrackerState::TEMP_LOST;
            lost_count_ = 1;
        } else {
            lost_count_ = 0;
        }
    } else if (tracker_state == TrackerState::TEMP_LOST) {
        if (!matched) {
            if (++lost_count_ > lost_thres) {
                tracker_state = TrackerState::LOST;
            }
        } else {
            tracker_state = TrackerState::TRACKING;
            lost_count_ = 0;
        }
    }
}

// -------------------------------------------------------------
// 弹道打靶法 (重力与空气阻力积分)
// -------------------------------------------------------------
std::tuple<double, double, bool> Tracker::solve_ballistic(RmTF& tf, const Armor& muzzle_target,
                                                        const Eigen::Vector3d& cam_to_gun_rpy,
                                                        const Eigen::Vector3d& imu_rpy) {
    if (bullet_speed < 1e-3) return {0.0, 0.0, false};

    double x = muzzle_target.pos(0), y = muzzle_target.pos(1), z = muzzle_target.pos(2);
    double dist_h = std::hypot(x, y);

    if (dist_h < 0.1 || dist_h > 12.0 || std::isnan(dist_h)) return {0.0, 0.0, false};

    double v_init = bullet_speed;
    const double g = 9.81;
    double pitch_rad = std::atan2(z, dist_h); // 初始瞄准角度猜测

    // 5 次打靶逼近迭代
    for (int i = 0; i < 5; ++i) {
        double sim_x = 0.0, sim_z = 0.0;
        double v_x = v_init * std::cos(pitch_rad);
        double v_z = v_init * std::sin(pitch_rad);
        double t = 0.0;
        const double dt_sim = 0.005;

        while (sim_x < dist_h && t < 2.0) {
            double v = std::hypot(v_x, v_z);
            double a_x = -k_v2 * v * v_x;
            double a_z = -g - k_v2 * v * v_z;

            sim_x += v_x * dt_sim;
            sim_z += v_z * dt_sim;
            v_x += a_x * dt_sim;
            v_z += a_z * dt_sim;
            t += dt_sim;
        }

        double z_error = z - sim_z;
        if (std::abs(z_error) < 0.005) break; // 误差小于 5mm，认为命中
        pitch_rad += z_error / dist_h; // 比例调节
    }

    // 虚拟无重力瞄准点映射回世界系
    double z_aim = dist_h * std::tan(pitch_rad);
    Eigen::Vector3d aim_point_world(x, y, z_aim);

    // 调用 TF 转回相机系 (伪代码，调用你的 TF 模块)
    Eigen::Vector3d aim_point_cam = tf.world_to_cam(aim_point_world, imu_rpy);

    double delta_yaw = std::atan2(aim_point_cam(0), aim_point_cam(2));
    double delta_pitch = std::atan2(aim_point_cam(1), aim_point_cam(2));

    // 外参机械补偿
    delta_yaw += cam_to_gun_rpy(2) * DEG2RAD;
    delta_pitch += cam_to_gun_rpy(1) * DEG2RAD;

    return {delta_yaw, delta_pitch, true};
}

// =====================================================================
// 属于 Tracker 类作用域的主时间轴解算管线函数
// =====================================================================
std::tuple<std::vector<double>, std::vector<std::pair<std::string, std::string>>>
Tracker::track(RmTF& tf, const std::vector<Armor>& raw_armors, const Eigen::Vector3d& imu_rpy, double dt) {

    std::vector<std::pair<std::string, std::string>> logs;
    std::vector<double> gimbal_control = {0.0, 0.0, 0.0}; // 映射：[yaw_deg, pitch_deg, can_fire_flag]

    // 1. 判断并调度基础状态机
    if (tracker_state == TrackerState::LOST) {
        if (!raw_armors.empty()) {
            // 目标发现：直接捕获第一块进入视野的装甲板初始化卡尔曼状态核
            tracked_id = raw_armors[0].id;
            init_ekf(raw_armors[0]);
            tracker_state = TrackerState::DETECTING;
            detect_count_ = 1;
        }
        return {gimbal_control, logs};
    } else {
        // 如果处于追踪期：灌入 EKF 进行多阶导数更新
        update(raw_armors, dt);
    }

    // 兜底机制：若更新判定目标大范围离群导致状态机瞬间跌落，紧急刹车退出
    if (tracker_state == TrackerState::LOST) {
        tracked_id = -1;
        return {gimbal_control, logs};
    }

    // 2. 时空转换与多阶卡尔曼预测：提取当前预测的目标中心在世界系下的绝对三维坐标
    Eigen::Vector3d current_armor_pos = get_armor_position_from_state(target_state_);
    Armor virtual_target(tracked_id, current_armor_pos(0), current_armor_pos(1), current_armor_pos(2), target_state_(6));

    // 3. 枪口偏置补偿：通过调用我们的高精 C++ 坐标系变换工具，剥离机械外参
    Eigen::Vector3d offset_world = tf.cam_to_world(cam_to_gun_pos, imu_rpy);
    virtual_target.pos -= offset_world;

    // 4. 空气阻力积分：启动五阶打靶法迭代，逆向追溯获取真实无重力虚拟瞄准偏角
    auto [yaw_cmd, pitch_cmd, isValid] = solve_ballistic(tf, virtual_target, cam_to_gun_rpy, imu_rpy);

    // 5. 电控发弹许可逻辑决策
    bool can_fire = false;
    if (isValid && std::abs(yaw_cmd) < yaw_tolerance_deg && std::abs(pitch_cmd) < pitch_tolerance_deg) {
        can_fire = true; // 只有当枪口收敛误差低于误差死区容忍值时，才拉高允许发弹电平信号
    }

    // 装填产物
    gimbal_control[0] = yaw_cmd;
    gimbal_control[1] = pitch_cmd;
    gimbal_control[2] = can_fire ? 1.0 : 0.0;

    return {gimbal_control, logs};
}

} // namespace dt46_vision
