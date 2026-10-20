#include "rm_tracker/tracker.hpp"
#include <algorithm>
#include <string>

namespace dt46_vision {

Tracker::Tracker() {
    tracker_state = TrackerState::LOST;
    tracked_id = -1;
    target_state_.setZero();

    robot_list_.emplace_back(0, 135.0, 125.0, 0.3, 0.285);
    robot_list_.emplace_back(1, 135.0, 125.0, 0.23, 0.21);
    robot_list_.emplace_back(2, 135.0, 125.0, 0.3, 0.27);
    robot_list_.emplace_back(3, 135.0, 125.0, 0.23, 0.21);
    robot_list_.emplace_back(4, 135.0, 125.0, 0.23, 0.21);
    robot_list_.emplace_back(5, 135.0, 125.0, 0.24, 0.22);
    robot_list_.emplace_back(6, 135.0, 125.0, 0.3, 0.285);
    robot_list_.emplace_back(7, 135.0, 125.0, 0.23, 0.21);
    // detector 对红板给 6/8/11、蓝板给 0/2/5；这里补上红方步兵/哨兵，几何取蓝方同类的值。
    // 缺这两条时 init_ekf 拿不到表值，r_init 退到 0.26、another_r_ 用残值 —— 实测 id8 的 r 会漂到 0.40 clamp。
    robot_list_.emplace_back(8, 135.0, 125.0, 0.30, 0.27);
    robot_list_.emplace_back(11, 135.0, 125.0, 0.24, 0.22);

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

// 平面 PnP 的两个解关于视线镜像：实测 (yaw0+yaw1)/2 + 板方位角 在各数据集里是常数（std 2~7°），
// 而两解本身差 20~70°。位置观测没有这个二义，所以用它挑分支；被选中的那个值仍保留 PnP 的低噪声
// （镜像式里的板方位角 atan2(y,x) 精度约 1°）。带 15° 迟滞，避免在两者接近时来回抖。
double Tracker::disambiguate_armor_yaw(double obs_yaw, const Eigen::Vector3d& armor_pos,
                                       double ref_yaw, double ref_x, double ref_y) const {
    const double yaw_a = orientation_to_yaw(obs_yaw, ref_yaw);
    const double bearing = std::atan2(armor_pos(1), armor_pos(0)); // 相机(世界原点)->板 的方位角
    const double yaw_b = 2.0 * bearing - yaw_a;                    // 绕视线的镜像解

    const double dx = ref_x - armor_pos(0), dy = ref_y - armor_pos(1);
    if (std::hypot(dx, dy) < 1e-3) return yaw_a;                   // 没参考心时不动

    const double geom_yaw = std::atan2(dy, dx);                    // 板 -> 心 的几何方向
    const double d_a = std::abs(shortest_angular_distance(geom_yaw, yaw_a));
    const double d_b = std::abs(shortest_angular_distance(geom_yaw, yaw_b));
    return (d_b + 15.0 * DEG2RAD < d_a) ? yaw_b : yaw_a;
}

void Tracker::resolve_center_from_armor(double xa, double ya, double r, double& yaw,
                                        double& xc, double& yc,
                                        bool has_prev, double prev_xc, double prev_yc) const {
    // yaw 指向「板 → 心」；π 二义对应心在板的两侧，只能二选一
    const double c1x = xa + r * std::cos(yaw);
    const double c1y = ya + r * std::sin(yaw);
    const double c2x = xa - r * std::cos(yaw);
    const double c2y = ya - r * std::sin(yaw);

    bool pick_c2 = false;
    if (has_prev) {
        const double d1 = std::hypot(c1x - prev_xc, c1y - prev_yc);
        const double d2 = std::hypot(c2x - prev_xc, c2y - prev_yc);
        // 贴近上一帧车心；两者差不多时选离相机更远的一侧（正对时心在板后）
        if (d2 + 0.02 < d1) {
            pick_c2 = true;
        } else if (d1 + 0.02 < d2) {
            pick_c2 = false;
        } else {
            pick_c2 = (c2x * c2x + c2y * c2y) > (c1x * c1x + c1y * c1y);
        }
    } else {
        // 无先验：正对锁板时心应比板更远
        pick_c2 = (c2x * c2x + c2y * c2y) > (c1x * c1x + c1y * c1y);
    }

    if (pick_c2) {
        yaw = normalize_angle(yaw + M_PI);
        xc = c2x;
        yc = c2y;
    } else {
        xc = c1x;
        yc = c1y;
    }
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

    double xc = 0.0, yc = 0.0;
    resolve_center_from_armor(xa, ya, r_init, last_yaw_, xc, yc, false, 0.0, 0.0);

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
    center_hist_.clear();
}
void Tracker::reset_plate(const Armor& armor) {
    plate_.setZero();
    plate_(0) = armor.pos(0);
    plate_(2) = armor.pos(1);
    plate_(4) = armor.pos(2);
    plate_inited_ = true;
    lead_vel_.setZero();
    plate_meas_ = armor;
    last_plate_yaw_ = armor.yaw;
    spin_ = false;
    spin_jump_count_ = 0;
    last_jump_sign_ = 0;
    spin_quiet_ = 0.0;
    plate_move_speed_ = 0.0;
    move_hist_.clear();
    center_hist_.clear();
    plate_arc_.clear();
    spin_center_valid_ = false;
    fit_hist_.clear();
    spin_periods_.clear();
    last_same_jump_clock_ = -1.0;
    move_clock_ = 0.0;
    last_jump_clock_ = -1.0;
    ekf_reject_streak_ = 0;
}

void Tracker::note_plate_jump(const Eigen::Vector3d& pred, const Eigen::Vector3d& meas,
                              std::vector<std::pair<std::string, std::string>>& logs) {
    // 跳变相对相机方位的左右符号。同号连续两次才当小陀螺，单次深度或误检不会置位。
    const double cross = pred(0) * (meas(1) - pred(1)) - pred(1) * (meas(0) - pred(0));
    const int sign = (cross > 0.0) ? 1 : (cross < 0.0) ? -1 : 0;
    if (sign == 0) return;
    if (sign == last_jump_sign_) {
        spin_jump_count_++;
        // 同向换板说明还在转。窗首尾都是朝向相机的板，原地小陀螺的窗位移接近 0。
        // 换向的单次毛刺不刷新，否则停住后一次误检就把退出计时清掉。
        spin_quiet_ = 0.0;
        if (last_same_jump_clock_ >= 0.0) {
            const double period = move_clock_ - last_same_jump_clock_;
            // 太短是同一次换板被记了两下，太长不是连续的下一块板。
            if (period >= 0.15 && period <= 1.0) {
                spin_periods_.push_back(period);
                while (spin_periods_.size() > 3) spin_periods_.pop_front();
            }
        }
        last_same_jump_clock_ = move_clock_;
    } else {
        last_jump_sign_ = sign;
        spin_jump_count_ = 1;
        spin_periods_.clear();
        last_same_jump_clock_ = move_clock_;
    }
    if (spin_jump_count_ >= 2 && !spin_) {
        spin_ = true;
        logs.emplace_back("spin", "SPIN MODE ON (same-direction plate jump)");
    }
}

void Tracker::spin_quiet_tick(double dt, bool jumped,
                              std::vector<std::pair<std::string, std::string>>& logs) {
    (void)jumped;
    // 速度是 0.4s 窗位移，不是帧间差分。停住时窗位移接近 0，约 1s 退出。
    // 同向换板在 note_plate_jump 里把计时清掉，原地小陀螺不会因为窗位移小而退出。
    if (plate_move_speed_ > 1.5) {
        spin_quiet_ = 0.0;
        return;
    }
    spin_quiet_ += dt;
    const double need = plate_move_speed_ < 0.4 ? 1.0 : kSpinQuiet;
    if (spin_quiet_ <= need) return;
    spin_jump_count_ = 0;
    spin_quiet_ = 0.0;
    if (spin_) {
        spin_ = false;
        spin_periods_.clear();
        last_same_jump_clock_ = -1.0;
        plate_arc_.clear();
        spin_center_valid_ = false;
        fit_hist_.clear();
        logs.emplace_back("spin", "SPIN MODE OFF (no plate jump)");
    }
}

void Tracker::note_center_samples() {
    center_hist_.push_back({move_clock_, target_state_(0), target_state_(2)});
    while (!center_hist_.empty() && move_clock_ - center_hist_.front().t > 0.5) {
        center_hist_.pop_front();
    }
}

void Tracker::note_plate_arc(double x, double y) {
    if (!plate_arc_.empty() && move_clock_ - plate_arc_.back().t > 0.15) {
        plate_arc_.clear();
    }
    plate_arc_.push_back({move_clock_, x, y});
    while (!plate_arc_.empty() && move_clock_ - plate_arc_.front().t > 0.8) {
        plate_arc_.pop_front();
    }
}

bool Tracker::center_motion_is_slow() const {
    if (center_hist_.size() < 2) return true;
    const double newest = center_hist_.back().t;
    const XySample* old = nullptr;
    for (auto it = center_hist_.rbegin(); it != center_hist_.rend(); ++it) {
        if (newest - it->t >= 0.4) {
            old = &(*it);
            break;
        }
    }
    if (old == nullptr) return true;
    return std::hypot(center_hist_.back().x - old->x, center_hist_.back().y - old->y) < 0.15;
}

void Tracker::hold_center_when_stationary() {
    if (!spin_ || center_hist_.size() < 2) return;
    // 一帧车心跳过 6 cm 不是车在平移。转过约 115° 的板位置平均值在圆心前方约 25 cm，不能拿来当车心。
    const auto& newest = center_hist_.back();
    const auto& prev = center_hist_[center_hist_.size() - 2];
    if (std::hypot(newest.x - prev.x, newest.y - prev.y) > 0.06) {
        target_state_(0) = prev.x;
        target_state_(2) = prev.y;
        center_hist_.back().x = prev.x;
        center_hist_.back().y = prev.y;
        target_state_(1) = 0.0;
        target_state_(3) = 0.0;
        target_state_(5) = 0.0;
    }
    // 匀速时检测几乎总是正前方那块板，板的位置挤在一小团里，圆拟合定不出圆心。
    // 正对相机的板在车心和相机之间，车心就在这块板的延长线上。侧面的板更远，不拿来改车心。
    if (plate_meas_.has_value()) {
        const double px = plate_meas_->pos(0);
        const double py = plate_meas_->pos(1);
        const double plate_dist = std::hypot(px, py);
        std::vector<double> dists;
        dists.reserve(plate_arc_.size());
        for (const auto& sample : plate_arc_) {
            dists.push_back(std::hypot(sample.x, sample.y));
        }
        std::sort(dists.begin(), dists.end());
        const double mid = dists[dists.size() / 2];
        // 用中位数。窗里最近的一帧当基准，一次测近就会把后面的板全丢掉，瞄准点就停住再跳。
        if (plate_dist > 0.3 && std::abs(plate_dist - mid) <= 0.15) {
            const double radius = std::max(target_state_(8), 0.2);
            const double scale = 1.0 + radius / plate_dist;
            spin_center_x_ = px * scale;
            spin_center_y_ = py * scale;
            spin_center_valid_ = true;
        }
    }
    if (spin_center_valid_) {
        target_state_(0) = spin_center_x_;
        target_state_(2) = spin_center_y_;
        center_hist_.back().x = spin_center_x_;
        center_hist_.back().y = spin_center_y_;
        fit_hist_.push_back({move_clock_, spin_center_x_, spin_center_y_});
        while (!fit_hist_.empty() && move_clock_ - fit_hist_.front().t > 0.5) {
            fit_hist_.pop_front();
        }
        const double span = fit_hist_.back().t - fit_hist_.front().t;
        const double moved = std::hypot(fit_hist_.back().x - fit_hist_.front().x,
                                        fit_hist_.back().y - fit_hist_.front().y);
        if (span < 0.4 || moved < 0.15) {
            target_state_(1) = 0.0;
            target_state_(3) = 0.0;
            target_state_(5) = 0.0;
        } else {
            target_state_(1) = (fit_hist_.back().x - fit_hist_.front().x) / span;
            target_state_(3) = (fit_hist_.back().y - fit_hist_.front().y) / span;
            target_state_(5) = 0.0;
        }
        return;
    }
    if (!center_motion_is_slow()) return;
    target_state_(1) = 0.0;
    target_state_(3) = 0.0;
    target_state_(5) = 0.0;
}

void Tracker::apply_spin_period() {
    if (!spin_ || spin_periods_.empty() || last_jump_sign_ == 0) {
        target_state_(7) = 0.0;
        return;
    }
    std::vector<double> periods(spin_periods_.begin(), spin_periods_.end());
    std::sort(periods.begin(), periods.end());
    const std::size_t mid = periods.size() / 2;
    const double median = (periods.size() % 2 == 1)
                              ? periods[mid]
                              : 0.5 * (periods[mid - 1] + periods[mid]);
    // 相邻板差 90°。符号是换板相对视线的左右，和板的切向一致。
    target_state_(7) = last_jump_sign_ * (M_PI / 2.0) / median;
}

void Tracker::update_lead_velocity(double dt) {
    // 位移窗口 0.15s。再叠一个同样长的低通，速度变化还是要 0.3s 才进提前量。
    const double a = dt / (0.05 + dt);
    const MoveSample* old = nullptr;
    for (auto it = move_hist_.rbegin(); it != move_hist_.rend(); ++it) {
        if (move_clock_ - it->t >= 0.15) {
            old = &(*it);
            break;
        }
    }
    if (old == nullptr) {
        if (move_hist_.size() < 2) lead_vel_ *= (1.0 - a);
        return;
    }
    const double span = move_clock_ - old->t;
    const bool jump_in_window = last_jump_clock_ >= old->t;
    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    if (!jump_in_window) {
        Eigen::Vector3d delta = move_hist_.back().pos - old->pos;
        delta.z() = 0.0;
        Eigen::Vector3d sight = move_hist_.back().pos;
        sight.z() = 0.0;
        if (sight.norm() > 1e-3) {
            const Eigen::Vector3d radial = sight.normalized();
            delta -= radial * delta.dot(radial);
        }
        target = delta / span;
    }
    lead_vel_ += a * (target - lead_vel_);
}

void Tracker::note_plate_motion(const Eigen::Vector3d& pos, double dt, bool jumped) {
    // 用约 0.4s 的位移判断车还在不在动。帧间速度会被 1cm 的深度毛刺放大到 1m/s 以上，停住退不出 SPIN。
    move_clock_ += dt;
    if (jumped) last_jump_clock_ = move_clock_;
    move_hist_.push_back({move_clock_, pos});
    while (move_hist_.size() > 1 && move_clock_ - move_hist_.front().t > kMoveWindow) {
        move_hist_.pop_front();
    }
    const double span = move_clock_ - move_hist_.front().t;
    if (span < 0.25 || move_hist_.size() < 2) return;
    const Eigen::Vector3d delta = pos - move_hist_.front().pos;
    plate_move_speed_ = std::hypot(delta.x(), delta.y()) / span;
}

void Tracker::update_plate(const std::vector<Armor>& armors, double dt,
                           std::vector<std::pair<std::string, std::string>>& logs) {
    if (!plate_inited_) return;

    plate_(0) += plate_(1) * dt;
    plate_(2) += plate_(3) * dt;
    plate_(4) += plate_(5) * dt;
    const Eigen::Vector3d pred(plate_(0), plate_(2), plate_(4));

    double best = std::numeric_limits<double>::infinity();
    std::optional<Armor> match;
    for (const auto& armor : armors) {
        if (armor.id != tracked_id) continue;
        const double dist = (pred - armor.pos).norm();
        if (dist < best) {
            best = dist;
            match = armor;
        }
    }

    const auto decay_velocity = [this, dt]() {
        const double decay = std::exp(-dt / 0.2);
        plate_(1) *= decay;
        plate_(3) *= decay;
        plate_(5) *= decay;
    };

    if (!match.has_value() || best > kSameRobot) {
        decay_velocity();
        move_clock_ += dt;
        update_lead_velocity(dt);
        plate_move_speed_ *= std::exp(-dt / 0.3);
        spin_quiet_tick(dt, false, logs);
        plate_meas_.reset();
        return;
    }

    const Eigen::Vector3d delta = match->pos - pred;
    const double pred_norm = pred.norm();
    double tangential = delta.norm();
    if (pred_norm > 1e-3) {
        const Eigen::Vector3d radial = pred / pred_norm;
        tangential = (delta - radial * delta.dot(radial)).norm();
    }

    // 换板是切向跳，深度噪声沿视线。把跳变写进速度，提前量会飞出去。
    const bool jumped = tangential > kJumpTangential;
    if (jumped) {
        plate_(0) = match->pos(0);
        plate_(2) = match->pos(1);
        plate_(4) = match->pos(2);
        note_plate_jump(pred, match->pos, logs);
    } else {
        // 深度毛刺沿视线。整段限幅会把切向平移一起缩小，所以速度只吃垂直于视线的水平分量。
        const double pos_gain = 0.55;
        const double vel_gain = 6.0;
        const double vel_cap = 0.15;
        Eigen::Vector3d vel_delta = delta;
        if (pred_norm > 1e-3) {
            const Eigen::Vector3d radial = pred / pred_norm;
            vel_delta -= radial * vel_delta.dot(radial);
        }
        vel_delta.z() = 0.0;
        if (vel_delta.norm() > vel_cap) vel_delta *= vel_cap / vel_delta.norm();
        plate_(0) += pos_gain * delta(0);
        plate_(2) += pos_gain * delta(1);
        plate_(4) += pos_gain * delta(2);
        plate_(1) = std::clamp(plate_(1) + vel_gain * vel_delta(0), -15.0, 15.0);
        plate_(3) = std::clamp(plate_(3) + vel_gain * vel_delta(1), -15.0, 15.0);
        plate_(5) = std::clamp(plate_(5) * std::exp(-dt / 0.2), -2.0, 2.0);
    }

    plate_meas_ = *match;
    last_plate_yaw_ = orientation_to_yaw(match->yaw, last_plate_yaw_);
    note_plate_motion(match->pos, dt, jumped);
    update_lead_velocity(dt);
    spin_quiet_tick(dt, jumped, logs);
}

void Tracker::lock_target(const Armor& armor) {
    tracked_id = armor.id;
    init_ekf(armor);
    reset_plate(armor);
    tracker_state = TrackerState::DETECTING;
    detect_count_ = 1;
}

void Tracker::handle_armor_jump(const Armor& current_armor) {
    // 换板只把朝向拨过 90°。车心和速度留在原地，位置残差交给后面的装甲板更新。
    const double prev_yaw = target_state_(6);
    const int v_yaw_sign = (target_state_(7) >= 0.0) ? 1 : -1;
    int sign = v_yaw_sign;
    double expected = prev_yaw - sign * (M_PI / 2.0);
    double obs = orientation_to_yaw(current_armor.yaw, expected);
    const double err = std::abs(shortest_angular_distance(expected, obs));
    const double other = prev_yaw + sign * (M_PI / 2.0);
    const double obs_other = orientation_to_yaw(current_armor.yaw, other);
    const double err_other = std::abs(shortest_angular_distance(other, obs_other));
    if (err_other + 0.15 < err) {
        expected = other;
    }

    const double raw_dz = target_state_(4) - current_armor.pos(2);
    dz_ = std::clamp(raw_dz, -0.085, 0.085);
    target_state_(4) = current_armor.pos(2);
    std::swap(target_state_(8), another_r_);
    target_state_(6) = normalize_angle(expected);

    ekf_.setState(target_state_);
    ekf_.smooth_reset_covariance();
}

bool Tracker::apply_armor_measurement(const Armor& armor) {
    const double prev_vyaw = target_state_(7);
    const double yaw_meas = orientation_to_yaw(armor.yaw, target_state_(6));
    const double yaw_step = std::abs(shortest_angular_distance(target_state_(6), yaw_meas));
    const Eigen::Vector4d measurement(armor.pos(0), armor.pos(1), armor.pos(2), yaw_meas);
    if (!ekf_.update(measurement)) return false;
    target_state_ = ekf_.getState();
    // 一次接近 90° 的朝向纠正不是转速。
    if (yaw_step > 0.5) {
        target_state_(7) = prev_vyaw;
    }
    return true;
}

void Tracker::update(const std::vector<Armor>& armors, double dt, std::vector<std::pair<std::string, std::string>>& logs) {
    if (jump_cooldown_ > 0) jump_cooldown_--;

    update_plate(armors, dt, logs);

    ekf_.predict(dt);
    target_state_ = ekf_.getState();

    bool ekf_matched = false;
    std::optional<Armor> best_match = std::nullopt;
    double min_position_diff = std::numeric_limits<double>::infinity();
    double yaw_diff = std::numeric_limits<double>::infinity();

    Eigen::Vector3d pred_armor_pos = get_armor_position_from_state(target_state_);

    double best_match_yaw = 0.0;
    if (!armors.empty()) {
        for (const auto& armor : armors) {
            if (armor.id == tracked_id) {
                double p_diff = (pred_armor_pos - armor.pos).norm();
                if (p_diff < min_position_diff) {
                    min_position_diff = p_diff;
                    // 先用位置消掉 PnP 板 yaw 的镜像二义，再算 yaw 差，避免镜像解触发"换板"
                    best_match_yaw = disambiguate_armor_yaw(armor.yaw, armor.pos, target_state_(6),
                                                            target_state_(0), target_state_(2));
                    yaw_diff = std::abs(shortest_angular_distance(target_state_(6), best_match_yaw));
                    best_match = armor;
                }
            }
        }

        if (best_match.has_value()) {
            // 换板：观测要更靠近另一块预测板，而不是仅仅「离当前板超过匹配距离」。
            // 车心到本车任意一块板都约等于 r，用车心距离当门会把一次位置噪声当成换板。
            double nearest_other = std::numeric_limits<double>::infinity();
            const double yaw = target_state_(6);
            const double xc = target_state_(0), yc = target_state_(2), za = target_state_(4);
            for (int i = 1; i < 4; ++i) {
                const double armor_yaw = yaw - i * (M_PI / 2.0);
                const double r = (i % 2 == 0) ? target_state_(8) : another_r_;
                const double z = (i % 2 == 0) ? za : (za + dz_);
                const Eigen::Vector3d other(xc - r * std::cos(armor_yaw),
                                            yc - r * std::sin(armor_yaw), z);
                nearest_other = std::min(nearest_other, (other - best_match->pos).norm());
            }
            const bool switched_plate = nearest_other + 0.05 < min_position_diff
                                     && nearest_other < max_match_distance;

            if (min_position_diff < max_match_distance && yaw_diff < max_match_yaw_diff) {
                ekf_matched = apply_armor_measurement(*best_match);
            } else if (switched_plate && jump_cooldown_ == 0) {
                handle_armor_jump(*best_match);
                jump_cooldown_ = jump_cooldown_max;
                ekf_matched = true;
                apply_armor_measurement(*best_match);
            }
        }
    }

    const bool radius_railed = target_state_(8) < radius_params.r_min || target_state_(8) > radius_params.r_max;
    // 钳制状态范围防发散
    target_state_(8) = std::clamp(target_state_(8), radius_params.r_min, radius_params.r_max);
    target_state_(1) = std::clamp(target_state_(1), -15.0, 15.0);
    target_state_(3) = std::clamp(target_state_(3), -15.0, 15.0);
    target_state_(5) = std::clamp(target_state_(5), -2.0, 2.0);

    // 车心跟不上这块板时丢掉 9 维状态，用当前板重新初始化。板自己的速度留着继续瞄。
    const bool plate_ok = plate_meas_.has_value();
    if (plate_ok && (!ekf_matched || radius_railed)) {
        if (++ekf_reject_streak_ >= 5) {
            // 连续拒绝会整状态重开，转速被写成 0。小陀螺还在转，这一下不是停转。
            const double kept_vyaw = spin_ ? target_state_(7) : 0.0;
            const double kept_x = target_state_(0);
            const double kept_y = target_state_(2);
            init_ekf(*plate_meas_);
            if (spin_) {
                target_state_(0) = kept_x;
                target_state_(2) = kept_y;
                target_state_(1) = 0.0;
                target_state_(3) = 0.0;
                target_state_(7) = kept_vyaw;
            }
            ekf_reject_streak_ = 0;
            logs.emplace_back("state", "EKF reset from plate");
        }
    } else if (ekf_matched) {
        ekf_reject_streak_ = 0;
    }

    // 滑行期只衰减线速度：掉帧时避免平移外推跑飞；v_yaw 保留给小陀螺换板
    if (!ekf_matched) {
        const double decay = std::exp(-dt / 0.2); // 时间常数 0.2s，帧率无关
        target_state_(1) *= decay;
        target_state_(3) *= decay;
        target_state_(5) *= decay;
    }
    note_center_samples();
    if (plate_meas_.has_value()) {
        note_plate_arc(plate_meas_->pos(0), plate_meas_->pos(1));
    }
    hold_center_when_stationary();
    apply_spin_period();
    ekf_.setState(target_state_);

    // 状态机跟板的关联走，不跟车心滤波。车心拒绝更新时检测还在，不该因此丢失。
    if (tracker_state == TrackerState::DETECTING) {
        if (plate_ok) {
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
        if (!plate_ok) {
            tracker_state = TrackerState::TEMP_LOST;
            lost_count_ = 1;
            logs.emplace_back("state", "TRACKING -> TEMP_LOST");
        } else {
            lost_count_ = 0;
        }
    } else if (tracker_state == TrackerState::TEMP_LOST) {
        if (!plate_ok) {
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

    double v_xc = current_state(1), v_yc = current_state(3), v_za = current_state(5);
    double v_yaw = current_state(7);

    // 子弹飞行时间：用带阻力的 LUT 查表，避免 dist/v0 少算阻力。
    // 用当前跟踪装甲板的实际位置（水平距离 + 相对高度），而不是 center_dist - r 的粗略下界。
    Eigen::Vector3d armor_pos = get_armor_position_from_state(current_state);
    double armor_dist_h = std::hypot(armor_pos(0), armor_pos(1));
    double t_flight = (armor_dist_h <= LUT_DIST_MAX)
                      ? lut_lookup_tof(armor_dist_h, armor_pos(2))
                      : armor_dist_h / bullet_speed;
    double t = t_flight + system_delay;

    // 线性外推未来状态。自旋时横向速度是板的切向残差，不拿去推车心
    Eigen::Matrix<double, 9, 1> future_state = current_state;
    if (!spin_) {
        future_state(0) += v_xc * t;
        future_state(2) += v_yc * t;
    }
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
std::optional<Armor> Tracker::find_target(std::vector<Armor>& robot_armors, const Eigen::Matrix<double, 9, 1>& state, double dt) {
    if (robot_armors.empty()) return std::nullopt;

    std::optional<Armor> target = std::nullopt;

    double xc = state(0), yc = state(2);
    double yaw_center_to_cam = std::atan2(-yc, -xc);
    (void)dt;

    // --- 选板决策 ---
    if (spin_) {
        // 选最朝向相机的那块预测板，击打点就用这块板的位置。
        // 按 yaw 正负切一半会在过 0 / ±π 时换板，朝相机的圆点又会把车心误差原样反映到击打点上。
        double min_angle_diff = std::numeric_limits<double>::infinity();
        for (auto& armor : robot_armors) {
            double yaw_center_to_armor = std::atan2(armor.pos(1) - yc, armor.pos(0) - xc);
            double angle_diff = std::abs(shortest_angular_distance(yaw_center_to_cam, yaw_center_to_armor));
            armor.angle_diff = angle_diff;
            if (angle_diff < min_angle_diff - (3.0 * DEG2RAD)) {
                min_angle_diff = angle_diff;
                target = armor;
            }
        }
        if (target.has_value()) {
            double r = (target->index % 2 == 0) ? state(8) : another_r_;
            Eigen::Vector3d front(xc - std::cos(yaw_center_to_cam + M_PI) * r,
                                  yc - std::sin(yaw_center_to_cam + M_PI) * r,
                                  target->pos(2));
            target->target_to_armor_dist = (target->pos - front).norm();
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

            if (angle_diff < min_angle_diff - (3.0 * DEG2RAD)) {
                min_angle_diff = angle_diff;
                target = armor;
            }
        }
    }

    // 距离统一用目标中心，而不是装甲板位置：目标旋转时装甲板距离会跟着 ±r 摆动
    if (target.has_value()) {
        target->dist = std::sqrt(xc * xc + yc * yc + state(4) * state(4));
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
    double armor_w = 0.135, armor_h = 0.125;
    for (const auto& r : robot_list_) {
        if (r.id == target.id) {
            armor_w = r.armor_width / 1000.0;
            armor_h = r.armor_height / 1000.0;
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
        // 侧向误差是到达时板心离前点的弧长。小于半宽，子弹到的时候板还在前点上。
        const double half_w = armor_w * 0.5 - 0.0085;
        fire_flag = (std::abs(yaw) < yaw_tol_mix &&
                     std::abs(pitch) < pitch_tol_mix * 0.8 &&
                     target.dist <= shootable_dist &&
                     target.target_to_armor_dist < half_w);
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
    const double sim_dt = 0.001; // 积分步长；比 5ms 细，保证飞行时间有 ms 级精度

    for (int i = 0; i < LUT_DIST_BINS; ++i) {
        for (int j = 0; j < LUT_HEIGHT_BINS; ++j) {
            double dist_h = LUT_DIST_MIN + i * LUT_DIST_STEP;
            double z_target = LUT_HEIGHT_MIN + j * LUT_HEIGHT_STEP;

            double pitch = std::atan2(z_target, dist_h);
            double tof = 0.0;

            for (int iter = 0; iter < 5; ++iter) {
                double sim_x = 0.0, sim_z = 0.0;
                double v_x = v_init * std::cos(pitch);
                double v_z = v_init * std::sin(pitch);
                double t = 0.0;

                while (sim_x < dist_h && t < 2.0) {
                    double v = std::hypot(v_x, v_z);
                    double a_x = -k_v2 * v * v_x;
                    double a_z = -g - k_v2 * v * v_z;

                    sim_x += v_x * sim_dt;
                    sim_z += v_z * sim_dt;
                    v_x += a_x * sim_dt;
                    v_z += a_z * sim_dt;
                    t += sim_dt;
                }
                tof = t;

                double z_error = z_target - sim_z;
                if (std::abs(z_error) < 0.005) break;
                pitch += z_error / std::max(dist_h, 0.1);
            }

            ballistic_lut_[i][j] = pitch;
            tof_lut_[i][j] = tof;
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

double Tracker::lut_lookup_tof(double dist_h, double z) const {
    double di = (dist_h - LUT_DIST_MIN) / LUT_DIST_STEP;
    double dj = (z - LUT_HEIGHT_MIN) / LUT_HEIGHT_STEP;

    int i0 = std::clamp(static_cast<int>(di), 0, LUT_DIST_BINS - 1);
    int j0 = std::clamp(static_cast<int>(dj), 0, LUT_HEIGHT_BINS - 1);
    int i1 = std::min(i0 + 1, LUT_DIST_BINS - 1);
    int j1 = std::min(j0 + 1, LUT_HEIGHT_BINS - 1);

    double fi = di - i0;
    double fj = dj - j0;

    return (1.0 - fi) * (1.0 - fj) * tof_lut_[i0][j0]
         + fi * (1.0 - fj) * tof_lut_[i1][j0]
         + (1.0 - fi) * fj * tof_lut_[i0][j1]
         + fi * fj * tof_lut_[i1][j1];
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

    if (dist_h < 0.2 || std::isnan(dist_h)) return {0.0, 0.0, false};

    if (dist_h < 1.5) {
        double pitch_rad = std::atan2(z, dist_h);
        double z_aim = dist_h * std::tan(pitch_rad);
        Eigen::Vector3d aim_point_world(x, y, z_aim);
        Eigen::Vector3d aim_point_cam = tf.world_to_cam(aim_point_world, imu_rpy);

        double delta_yaw = std::atan2(aim_point_cam(0), aim_point_cam(2));
        double delta_pitch = std::atan2(aim_point_cam(1), aim_point_cam(2));

        delta_yaw = std::clamp(delta_yaw, -1.0, 1.0) + cam_to_gun_rpy(2) * DEG2RAD;
        delta_pitch = std::clamp(delta_pitch, -1.0, 1.0) + cam_to_gun_rpy(1) * DEG2RAD;

        return {delta_yaw, delta_pitch, true};
    }

    double pitch_rad = dist_h <= LUT_DIST_MAX ? lut_lookup(dist_h, z) : std::atan2(z, dist_h);

    double z_aim = dist_h * std::tan(pitch_rad);
    Eigen::Vector3d aim_point_world(x, y, z_aim);

    Eigen::Vector3d aim_point_cam = tf.world_to_cam(aim_point_world, imu_rpy);

    double delta_yaw = std::atan2(aim_point_cam(0), aim_point_cam(2));
    double delta_pitch = std::atan2(aim_point_cam(1), aim_point_cam(2));

    delta_yaw = std::clamp(delta_yaw, -1.0, 1.0) + cam_to_gun_rpy(2) * DEG2RAD;
    delta_pitch = std::clamp(delta_pitch, -1.0, 1.0) + cam_to_gun_rpy(1) * DEG2RAD;

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

    // 平时瞄这块板自己的位置和速度。小陀螺改瞄到达时刻正对相机的那块板。
    if (!plate_inited_) {
        return {gimbal_control, logs};
    }
    Armor target_cam(tracked_id, 0.0, 0.0, 0.0, last_plate_yaw_);
    bool aim_at_front = false;
    const double center_norm = std::hypot(target_state_(0), target_state_(2));
    if (spin_ && center_norm > 0.3) {
        double xc = target_state_(0), yc = target_state_(2), za = target_state_(4);
        const double rough = std::max(0.1, center_norm - target_state_(8));
        const double t_flight = (rough <= LUT_DIST_MAX)
                                ? lut_lookup_tof(rough, za)
                                : rough / std::max(bullet_speed, 1e-3);
        const double lead_t = t_flight + system_delay;
        xc += target_state_(1) * lead_t;
        yc += target_state_(3) * lead_t;
        za += target_state_(5) * lead_t;
        const double yaw_arr = target_state_(6) + target_state_(7) * lead_t;
        const double arrived_norm = std::hypot(xc, yc);
        if (arrived_norm > 0.3) {
            const double front_yaw = std::atan2(yc, xc);
            const double delta = shortest_angular_distance(front_yaw, yaw_arr);
            const int plate_steps = static_cast<int>(std::lround(delta / (M_PI / 2.0)));
            const int plate_i = (plate_steps % 4 + 4) % 4;
            const double plate_err = std::abs(delta - plate_steps * (M_PI / 2.0));
            const bool even = (plate_i % 2 == 0);
            const double r = even ? target_state_(8) : another_r_;
            const double z = even ? za : (za + dz_);
            const double ux = xc / arrived_norm, uy = yc / arrived_norm;
            target_cam = Armor(tracked_id, xc - r * ux, yc - r * uy, z, front_yaw, plate_i);
            target_cam.target_to_armor_dist = r * plate_err;
            aim_at_front = true;
        }
    }
    if (!aim_at_front) {
        const Eigen::Vector3d plate_pos(plate_(0), plate_(2), plate_(4));
        const double dist_h = std::hypot(plate_pos(0), plate_pos(1));
        const double t_flight = (dist_h <= LUT_DIST_MAX)
                                ? lut_lookup_tof(dist_h, plate_pos(2))
                                : dist_h / std::max(bullet_speed, 1e-3);
        const double lead_t = t_flight + system_delay;
        const Eigen::Vector3d aim = plate_pos + lead_vel_ * lead_t;
        target_cam = Armor(tracked_id, aim(0), aim(1), aim(2), last_plate_yaw_);
        // 小陀螺但车心不可用时不打。误差写成大于半宽，开火门关着。
        target_cam.target_to_armor_dist = spin_ ? 1.0 : 0.0;
    }

    // 4. 转换至枪口系并打靶
    std::optional<Armor> target_muzzle = world_to_muzzle(tf, target_cam, cam_to_gun_pos, imu_rpy);
    if (!target_muzzle.has_value()) {
        return {gimbal_control, logs};
    }

    auto [yaw_cmd_rad, pitch_cmd_rad, isValid] = solve_ballistic(tf, target_muzzle.value(), cam_to_gun_rpy, imu_rpy);

    // 5. 动态发弹许可
    // 注意：solve_ballistic 返回的是弧度，但 can_fire 期望的是角度
    auto final_gimbal = can_fire(target_cam, {yaw_cmd_rad * RAD2DEG, pitch_cmd_rad * RAD2DEG, isValid});

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
