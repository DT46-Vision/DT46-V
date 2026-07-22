#include "rm_tracker/tracker.hpp"
#include <algorithm>
#include <string>

namespace dt46_vision {

Tracker::Tracker() {
    tracker_state = TrackerState::LOST;
    tracked_id = -1;
    target_state_.setZero();

    robot_list_.emplace_back(0, 230.0, 125.0, 0.3, 0.285);
    robot_list_.emplace_back(1, 135.0, 125.0, 0.23, 0.21);
    robot_list_.emplace_back(2, 135.0, 125.0, 0.3, 0.27);
    robot_list_.emplace_back(3, 135.0, 125.0, 0.23, 0.21);
    robot_list_.emplace_back(4, 135.0, 125.0, 0.23, 0.21);
    robot_list_.emplace_back(5, 135.0, 125.0, 0.24, 0.22);
    robot_list_.emplace_back(6, 230.0, 125.0, 0.3, 0.285);
    robot_list_.emplace_back(7, 135.0, 125.0, 0.23, 0.21);

    build_ballistic_lut();
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

void Tracker::try_init_tracker(std::vector<Armor>& armors) {
    if (armors.empty()) return;

    // 按 Z 轴深度升序排列 (从小到大，选最近的)
    std::sort(armors.begin(), armors.end(), [](const Armor& a, const Armor& b) {
        return a.pos(2) < b.pos(2);
    });

    Armor chosen_armor = armors[0];

    // 如果存在第二个目标，且两者深度差在 dist_tol 范围内，则优选 X 轴靠近中心的
    if (armors.size() >= 2) {
        Armor first = armors[0];
        Armor second = armors[1];

        if (std::abs(first.pos(2) - second.pos(2)) <= dist_tol) {
            if (std::abs(first.pos(0)) > std::abs(second.pos(0))) {
                chosen_armor = second;
            }
        }
    }

    lock_target(chosen_armor);
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

    // ================== 新增：Yaw 翻转校验 ==================
    double xc = xa + r_init * std::cos(last_yaw_);
    double yc = ya + r_init * std::sin(last_yaw_);

    double norm_a_sq = xa * xa + ya * ya;
    double norm_c_sq = xc * xc + yc * yc;

    // 如果装甲板比推算出的车体中心还要远，说明 Yaw 反了
    if (norm_a_sq > norm_c_sq) {
        // 翻转 Yaw 角 (加上 PI 并归一化)
        last_yaw_ = normalize_angle(last_yaw_ + M_PI);

        // 使用正确的 Yaw 重新计算中心点
        xc = xa + r_init * std::cos(last_yaw_);
        yc = ya + r_init * std::sin(last_yaw_);
    }
    // =======================================================

    // 注意这里的尖括号不能丢
    Eigen::Matrix<double, 9, 1> init_x;
    init_x << xc, 0, yc, 0, za, 0, last_yaw_, 0, r_init;

    Eigen::Matrix<double, 9, 9> init_p = Eigen::Matrix<double, 9, 9>::Identity();
    init_p(1,1) = 50.0; init_p(3,3) = 50.0; init_p(5,5) = 10.0; init_p(7,7) = 50.0;

    ekf_.init_QR(ekf_QR_params.q_xyz, ekf_QR_params.q_yaw, ekf_QR_params.q_r,
                 ekf_QR_params.r_xyz_factor, ekf_QR_params.r_yaw, ekf_QR_params.stable_dist);
    ekf_.init(init_x, init_p);
    target_state_ = init_x;
    dz_ = 0.0;
}
void Tracker::lock_target(const Armor& armor) {
    tracked_id = armor.id;
    init_ekf(armor);
    tracker_state = TrackerState::DETECTING;
    detect_count_ = 1;
}

void Tracker::handle_armor_jump(const Armor& current_armor) {
    double yaw = orientation_to_yaw(current_armor.yaw, target_state_(6));
    target_state_(6) = yaw;

    // 1. 高度钳制
    double raw_dz = target_state_(4) - current_armor.pos(2);
    dz_ = std::clamp(raw_dz, -0.085, 0.085);
    target_state_(4) = current_armor.pos(2);
    target_state_(5) = 0.0; // 斩断 Z 轴错误速度积分

    // 2. 速度衰减
    target_state_(1) *= 0.8;
    target_state_(3) *= 0.8;

    // 3. 半径交换
    std::swap(target_state_(8), another_r_);

    // 4. 【修复】：增加推算位置比对条件，防止无脑重置中心
    Eigen::Vector3d current_p = current_armor.pos;
    Eigen::Vector3d infer_p = get_armor_position_from_state(target_state_);

    if ((current_p - infer_p).norm() > max_match_distance) {
        double r = target_state_(8);
        double test_xc = current_p(0) + r * std::cos(yaw);
        double test_yc = current_p(1) + r * std::sin(yaw);

        // 法向量反向校验
        if ((current_p(0)*current_p(0) + current_p(1)*current_p(1)) > (test_xc*test_xc + test_yc*test_yc)) {
            yaw += M_PI;
            target_state_(6) = yaw;
        }

        target_state_(0) = current_p(0) + r * std::cos(yaw);
        target_state_(2) = current_p(1) + r * std::sin(yaw);
    }

    ekf_.setState(target_state_);
    ekf_.smooth_reset_covariance();
}

void Tracker::update(const std::vector<Armor>& armors, double dt, std::vector<std::pair<std::string, std::string>>& logs) {
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
                logs.emplace_back("state", "DETECTING -> TRACKING (ID: " + std::to_string(tracked_id) + ")");
            }
        } else {
            if (--detect_count_ <= 0) {
                tracker_state = TrackerState::LOST;
                logs.emplace_back("state", "DETECTING -> LOST (detect timeout)");
            }
        }
    } else if (tracker_state == TrackerState::TRACKING) {
        if (!matched) {
            tracker_state = TrackerState::TEMP_LOST;
            lost_count_ = 1;
            logs.emplace_back("state", "TRACKING -> TEMP_LOST");
        } else {
            lost_count_ = 0;
        }
    } else if (tracker_state == TrackerState::TEMP_LOST) {
        if (!matched) {
            if (++lost_count_ > lost_thres) {
                tracker_state = TrackerState::LOST;
                logs.emplace_back("state", "TEMP_LOST -> LOST (lost timeout)");
            }
        } else {
            tracker_state = TrackerState::TRACKING;
            lost_count_ = 0;
            logs.emplace_back("state", "TEMP_LOST -> TRACKING (recovered)");
        }
    }
}

// =====================================================================
// 1. 反迟滞预测：推算子弹到达时车体的整体未来状态
// =====================================================================
Eigen::Matrix<double, 9, 1> Tracker::predict_future_state() const {
    Eigen::Matrix<double, 9, 1> current_state = target_state_;
    if (bullet_speed < 1e-3) return current_state;

    double xc = current_state(0), yc = current_state(2), za = current_state(4);
    double v_xc = current_state(1), v_yc = current_state(3), v_za = current_state(5);
    double v_yaw = current_state(7);
    double r = current_state(8);

    // 估算子弹飞行时间 + 系统发弹延迟
    double center_dist = std::hypot(xc, yc, za);
    double rough_dist = std::max(0.1, center_dist - r);
    double t = (rough_dist / bullet_speed) + system_delay;

    // 线性外推未来状态
    Eigen::Matrix<double, 9, 1> future_state = current_state;
    future_state(0) += v_xc * t;
    future_state(2) += v_yc * t;
    future_state(4) += v_za * t;
    future_state(6) += v_yaw * t;

    return future_state;
}

// =====================================================================
// 2. 虚拟装甲板生成：依据未来中心位姿，生成 4 块车身装甲板
// =====================================================================
std::vector<Armor> Tracker::find_all_armors(const Eigen::Matrix<double, 9, 1>& state) const {
    double xc = state(0), yc = state(2), za = state(4);
    double yaw = state(6);
    double r1 = state(8);
    double r2 = another_r_;

    std::vector<Armor> robot_armors;
    for (int i = 0; i < 4; ++i) {
        double armor_yaw = yaw - i * (M_PI / 2.0);
        double r = (i % 2 == 0) ? r1 : r2;
        double z = (i % 2 == 0) ? za : (za + dz_);

        double ax = xc - r * std::cos(armor_yaw);
        double ay = yc - r * std::sin(armor_yaw);

        robot_armors.emplace_back(tracked_id, ax, ay, z, armor_yaw, i);
    }
    return robot_armors;
}

// =====================================================================
// 3. 小陀螺选板决策与抗抖动
// =====================================================================
std::optional<Armor> Tracker::find_target(std::vector<Armor>& robot_armors, const Eigen::Matrix<double, 9, 1>& state) {
    if (robot_armors.empty()) return std::nullopt;

    std::optional<Armor> best_armor = std::nullopt;
    std::optional<Armor> target = std::nullopt;
    double min_dist = std::numeric_limits<double>::infinity();

    double xc = state(0), yc = state(2);
    double yaw_center_to_cam = std::atan2(-yc, -xc);

    // --- 状态机：引入退出防抖，检测是否在打小陀螺 ---
    double current_yaw_vel = ekf_.getState()(7);
    if (std::abs(current_yaw_vel) > min_spinning_vel_) {
        min_spinning_frame_count_++;
        spinning_frame_lost_count_ = 0;
        if (min_spinning_frame_count_ > min_spinning_frame_) {
            min_spinning_frame_count_ = 0;
            spin_ = true;
        }
    } else {
        min_spinning_frame_count_ = 0;
        if (spin_) {
            spinning_frame_lost_count_++;
            if (spinning_frame_lost_count_ > spinning_frame_lost_) {
                spin_ = false;
                spinning_frame_lost_count_ = 0;
            }
        }
    }

    // --- 选板决策 ---
    if (spin_) {
        // 小陀螺模式：基于角速度方向过滤出“正向转过来”的那一半板
        std::vector<Armor> same_side;
        for (auto& armor : robot_armors) {
            double norm_yaw = normalize_angle(armor.yaw);
            if ((current_yaw_vel >= 0 && norm_yaw <= 0) || (current_yaw_vel < 0 && norm_yaw > 0)) {
                same_side.push_back(armor);
            }
        }

        for (auto& armor : same_side) {
            double dist = armor.pos.norm();
            double yaw_center_to_armor = std::atan2(armor.pos(1) - yc, armor.pos(0) - xc);
            armor.angle_diff = std::abs(shortest_angular_distance(yaw_center_to_cam, yaw_center_to_armor));

            if (dist < min_dist) {
                best_armor = armor;
                min_dist = dist;
            }
        }
        target = best_armor;

        // 【极度关键】：选定后，强制用平滑的圆方程覆盖其物理位置，彻底消除 PnP 抖动！
        if (target.has_value()) {
            double r = (target->index % 2 == 0) ? ekf_.getState()(8) : another_r_;
            target->pos(0) = xc - std::cos(yaw_center_to_cam + M_PI) * r;
            target->pos(1) = yc - std::sin(yaw_center_to_cam + M_PI) * r;
            target->target_to_armor_dist = (best_armor->pos - target->pos).norm();
        }
    } else {
        // 正常模式：取距离最近的两块板，选和相机光心夹角最小的那块（最正对的）
        std::sort(robot_armors.begin(), robot_armors.end(), [](const Armor& a, const Armor& b){
            return a.pos.norm() < b.pos.norm();
        });

        double min_angle_diff = std::numeric_limits<double>::infinity();
        size_t limit = std::min<size_t>(2, robot_armors.size());
        for (size_t i = 0; i < limit; ++i) {
            auto& armor = robot_armors[i];
            double yaw_center_to_armor = std::atan2(armor.pos(1) - yc, armor.pos(0) - xc);
            double angle_diff = std::abs(shortest_angular_distance(yaw_center_to_cam, yaw_center_to_armor));
            armor.angle_diff = angle_diff;

            if (angle_diff < min_angle_diff) {
                min_angle_diff = angle_diff;
                target = armor;
            }
        }
    }
    return target;
}

// =====================================================================
// 4. 世界坐标转枪口坐标 (为弹道积分准备)
// =====================================================================
std::optional<Armor> Tracker::world_to_muzzle(RmTF& tf, const Armor& target, const Eigen::Vector3d& offset_pos, const Eigen::Vector3d& imu_rpy) {
    Eigen::Vector3d pos_target_world = target.pos;
    Eigen::Vector3d offset_pos_world = tf.cam_to_world(offset_pos, imu_rpy);
    Eigen::Vector3d pos_gun_frame_world = pos_target_world - offset_pos_world;

    Armor muzzle_target(target.id, pos_gun_frame_world(0), pos_gun_frame_world(1), pos_gun_frame_world(2), target.yaw, target.index);
    muzzle_target.angle_diff = target.angle_diff;
    return muzzle_target;
}

// =====================================================================
// 5. 动态开火门限计算 (基于装甲板物理宽高等效映射)
// =====================================================================
std::tuple<double, double, bool> Tracker::can_fire(const Armor& target, std::tuple<double, double, bool> gimbal_control) {
    double yaw = std::get<0>(gimbal_control);
    double pitch = std::get<1>(gimbal_control);
    bool is_valid = std::get<2>(gimbal_control);

    if (!is_valid) return {yaw, pitch, false};

    // 获取当前装甲板的物理尺寸 (米)
    double armor_w = 0.135, armor_h = 0.125, armor_diagonal = 0.184;
    for (const auto& r : robot_list_) {
        if (r.id == target.id) {
            armor_w = r.armor_width / 1000.0;
            armor_h = r.armor_height / 1000.0;
            armor_diagonal = r.armor_diagonal / 1000.0;
            break;
        }
    }

    // 动态计算物理允许的角度偏差界限 (弧度转角度)
    double safe_scale = 0.8;
    double dynamic_yaw_tol = std::atan((armor_w / 2.0) / target.dist) * RAD2DEG * safe_scale;
    double dynamic_pitch_tol = std::atan((armor_h / 2.0) / target.dist) * RAD2DEG * safe_scale;

    double yaw_tol_mix = std::clamp(dynamic_yaw_tol, 1.0, 5.0);
    double pitch_tol_mix = std::clamp(dynamic_pitch_tol, 1.0, 5.0);
    this->yaw_tolerance_deg = yaw_tol_mix;
    this->pitch_tolerance_deg = pitch_tol_mix;

    bool fire_flag = false;
    if (spin_) {
        // 小陀螺模式：Pitch 卡得更严，且要求拟合位置偏差极小
        fire_flag = (std::abs(yaw) < yaw_tol_mix &&
                     std::abs(pitch) < pitch_tol_mix * 0.8 &&
                     target.dist <= shootable_dist &&
                     target.target_to_armor_dist <= (armor_diagonal / 2.0));
    } else {
        // 普通模式
        fire_flag = (std::abs(yaw) < yaw_tol_mix &&
                     std::abs(pitch) < pitch_tol_mix * 0.8 &&
                     target.dist <= shootable_dist);
    }

    return {yaw, pitch, fire_flag};
}

// =====================================================================
// 5.5 弹道查找表预计算与查询
// =====================================================================
void Tracker::build_ballistic_lut() {
    const double g = 9.81;
    const double v_init = bullet_speed > 0 ? bullet_speed : 28.0;

    for (int i = 0; i < LUT_DIST_BINS; ++i) {
        for (int j = 0; j < LUT_HEIGHT_BINS; ++j) {
            double dist_h = LUT_DIST_MIN + i * LUT_DIST_STEP;
            double z_target = LUT_HEIGHT_MIN + j * LUT_HEIGHT_STEP;

            double pitch = std::atan2(z_target, dist_h);

            for (int iter = 0; iter < 5; ++iter) {
                double sim_x = 0.0, sim_z = 0.0;
                double v_x = v_init * std::cos(pitch);
                double v_z = v_init * std::sin(pitch);
                double t = 0.0;

                while (sim_x < dist_h && t < 2.0) {
                    double v = std::hypot(v_x, v_z);
                    double a_x = -k_v2 * v * v_x;
                    double a_z = -g - k_v2 * v * v_z;

                    sim_x += v_x * 0.005;
                    sim_z += v_z * 0.005;
                    v_x += a_x * 0.005;
                    v_z += a_z * 0.005;
                    t += 0.005;
                }

                double z_error = z_target - sim_z;
                if (std::abs(z_error) < 0.005) break;
                pitch += z_error / std::max(dist_h, 0.1);
            }

            ballistic_lut_[i][j] = pitch;
        }
    }
}

double Tracker::lut_lookup(double dist_h, double z) const {
    double di = (dist_h - LUT_DIST_MIN) / LUT_DIST_STEP;
    double dj = (z - LUT_HEIGHT_MIN) / LUT_HEIGHT_STEP;

    int i0 = std::clamp(static_cast<int>(di), 0, LUT_DIST_BINS - 1);
    int j0 = std::clamp(static_cast<int>(dj), 0, LUT_HEIGHT_BINS - 1);
    int i1 = std::min(i0 + 1, LUT_DIST_BINS - 1);
    int j1 = std::min(j0 + 1, LUT_HEIGHT_BINS - 1);

    double fi = di - i0;
    double fj = dj - j0;

    return (1.0 - fi) * (1.0 - fj) * ballistic_lut_[i0][j0]
         + fi * (1.0 - fj) * ballistic_lut_[i1][j0]
         + (1.0 - fi) * fj * ballistic_lut_[i0][j1]
         + fi * fj * ballistic_lut_[i1][j1];
}

// -------------------------------------------------------------
// 6. 弹道打靶法 (重力与空气阻力积分)
// -------------------------------------------------------------
std::tuple<double, double, bool> Tracker::solve_ballistic(RmTF& tf, const Armor& muzzle_target,
                                                        const Eigen::Vector3d& cam_to_gun_rpy,
                                                        const Eigen::Vector3d& imu_rpy) {
    if (bullet_speed < 1e-3) return {0.0, 0.0, false};

    double x = muzzle_target.pos(0), y = muzzle_target.pos(1), z = muzzle_target.pos(2);
    double dist_h = std::hypot(x, y);

    if (dist_h < 0.1 || std::isnan(dist_h)) return {0.0, 0.0, false};

    double pitch_rad = dist_h <= LUT_DIST_MAX ? lut_lookup(dist_h, z) : std::atan2(z, dist_h);

    double z_aim = dist_h * std::tan(pitch_rad);
    Eigen::Vector3d aim_point_world(x, y, z_aim);

    Eigen::Vector3d aim_point_cam = tf.world_to_cam(aim_point_world, imu_rpy);

    double delta_yaw = std::atan2(aim_point_cam(0), aim_point_cam(2));
    double delta_pitch = std::atan2(aim_point_cam(1), aim_point_cam(2));

    delta_yaw += cam_to_gun_rpy(2) * DEG2RAD;
    delta_pitch += cam_to_gun_rpy(1) * DEG2RAD;

    return {delta_yaw, delta_pitch, true};
}

// =====================================================================
// 7. 属于 Tracker 类作用域的主时间轴解算管线函数
// =====================================================================
std::tuple<std::vector<double>, std::vector<std::pair<std::string, std::string>>>
Tracker::track(RmTF& tf, const std::vector<Armor>& raw_armors, const Eigen::Vector3d& imu_rpy, double dt) {

    // 【新增】：将原始观测数据存入缓存，供 UI 快照读取！
    debug_yaw_armors_cache_ = raw_armors;

    std::vector<std::pair<std::string, std::string>> logs;
    std::vector<double> gimbal_control = {0.0, 0.0, 0.0}; // 映射：[yaw_deg, pitch_deg, can_fire_flag]

    // 1. 判断并调度基础状态机
    if (tracker_state == TrackerState::LOST) {
        if (!raw_armors.empty()) {
            // 拷贝一份用于排序，避免破坏传入的 const raw_armors
            std::vector<Armor> temp_armors = raw_armors;
            try_init_tracker(temp_armors);
            logs.emplace_back("state", "LOST -> DETECTING (ID: " + std::to_string(tracked_id) + ")");
        }
        return {gimbal_control, logs};
    } else {
        // 如果处于追踪期：灌入 EKF 进行多阶导数更新
        update(raw_armors, dt, logs);
    }

    // 兜底机制：若更新判定目标大范围离群导致状态机瞬间跌落，紧急刹车退出
    if (tracker_state == TrackerState::LOST) {
        tracked_id = -1;
        logs.emplace_back("state", "TRACKING/TEMP_LOST -> LOST (target lost)");
        return {gimbal_control, logs};
    }

    // ==========================================================
    // 【主流程重构】
    // ==========================================================

    // 2. 时空转换与反迟滞预测
    Eigen::Matrix<double, 9, 1> future_state = predict_future_state();
    std::vector<Armor> robot_armors = find_all_armors(future_state);

    // 3. 小陀螺决策选板
    bool was_spin = spin_;
    std::optional<Armor> target_cam = find_target(robot_armors, future_state);
    if (was_spin != spin_) {
        logs.emplace_back("spin", spin_ ? "SPIN MODE ON (yaw_vel > threshold)" : "SPIN MODE OFF");
    }
    if (!target_cam.has_value()) {
        return {gimbal_control, logs};
    }

    // 4. 转换至枪口系并打靶
    std::optional<Armor> target_muzzle = world_to_muzzle(tf, target_cam.value(), cam_to_gun_pos, imu_rpy);
    if (!target_muzzle.has_value()) {
        return {gimbal_control, logs};
    }

    auto [yaw_cmd_rad, pitch_cmd_rad, isValid] = solve_ballistic(tf, target_muzzle.value(), cam_to_gun_rpy, imu_rpy);

    // 5. 动态发弹许可
    // 注意：solve_ballistic 返回的是弧度，但 can_fire 期望的是角度
    auto final_gimbal = can_fire(target_cam.value(), {yaw_cmd_rad * RAD2DEG, pitch_cmd_rad * RAD2DEG, isValid});

    // 【新增】：将刚刚算出的局部变量，固化存入类成员缓存中，给快照提供数据！
    target_cam_cache_ = target_cam;
    target_muzzle_cache_ = target_muzzle;
    gimbal_control_cache_ = final_gimbal;

    // 装填产物
    gimbal_control[0] = std::get<0>(final_gimbal); // Yaw 角度
    gimbal_control[1] = std::get<1>(final_gimbal); // Pitch 角度
    gimbal_control[2] = std::get<2>(final_gimbal) ? 1.0 : 0.0;

    return {gimbal_control, logs};
}

// =====================================================================
// 提取用于 UI 渲染的无锁快照
// =====================================================================
RenderSnapshot Tracker::get_render_snapshot() const {
    RenderSnapshot s;
    s.tracker_state = tracker_state;
    s.target_state = target_state_;
    s.another_r = another_r_;
    s.dz = dz_;
    s.debug_yaw_armors = debug_yaw_armors_cache_;
    s.target = target_cam_cache_;
    s.muzzle_target = target_muzzle_cache_;
    s.spin = spin_;
    s.yaw_tolerance_deg = yaw_tolerance_deg;
    s.pitch_tolerance_deg = pitch_tolerance_deg;
    s.gimbal_control = gimbal_control_cache_;
    s.ekf_yaw_vel = target_state_(7);
    s.bullet_speed = bullet_speed;
    return s;
}

} // namespace dt46_vision
