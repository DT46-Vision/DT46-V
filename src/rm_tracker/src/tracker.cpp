#include "rm_tracker/tracker.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
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
    obs_center_hist_.clear();
    plate_arc_.clear();
    spin_center_valid_ = false;
    spin_center_converged_ = false;
    last_center_accept_clock_ = -1.0;
    fit_hist_.clear();
    spin_periods_.clear();
    last_same_jump_clock_ = -1.0;
    move_clock_ = 0.0;
    last_jump_clock_ = -1.0;
    last_spin_vyaw_ = 0.0;
    spin_aim_translate_ = false;
    spin_aim_mode_clock_ = 0.0;
    spin_front_plate_i_ = 0;
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
        // 不钉单板反推的车心（常已横偏）；等双板或板轨迹拟合收敛后再信。
        spin_center_valid_ = false;
        spin_center_converged_ = false;
        last_center_accept_clock_ = -1.0;
        obs_center_hist_.clear();
        spin_aim_translate_ = false;
        spin_aim_mode_clock_ = 0.0;
        spin_front_plate_i_ = 0;
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
        spin_center_converged_ = false;
        last_center_accept_clock_ = -1.0;
        obs_center_hist_.clear();
        fit_hist_.clear();
        last_spin_vyaw_ = 0.0;
        spin_aim_translate_ = false;
        spin_aim_mode_clock_ = 0.0;
        spin_front_plate_i_ = 0;
        logs.emplace_back("spin", "SPIN MODE OFF (no plate jump)");
    }
}

void Tracker::note_plate_arc(double x, double y) {
    // 掉几帧别清空整段弧，否则拟合反复失锁、平移判别也跟着抖。
    if (!plate_arc_.empty() && move_clock_ - plate_arc_.back().t > 0.35) {
        plate_arc_.clear();
    }
    plate_arc_.push_back({move_clock_, x, y});
    while (!plate_arc_.empty() && move_clock_ - plate_arc_.front().t > 1.0) {
        plate_arc_.pop_front();
    }
}

bool Tracker::fit_center_from_plate_arc(double& xc, double& yc, double& radius) const {
    if (plate_arc_.size() < 16) return false;
    const double span = plate_arc_.back().t - plate_arc_.front().t;
    if (span < 0.35) return false;

    // Kåsa：x²+y² + D x + E y + F = 0 → 圆心 (-D/2,-E/2)
    const int n = static_cast<int>(plate_arc_.size());
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; ++i) {
        const double x = plate_arc_[static_cast<size_t>(i)].x;
        const double y = plate_arc_[static_cast<size_t>(i)].y;
        A(i, 0) = x;
        A(i, 1) = y;
        A(i, 2) = 1.0;
        b(i) = -(x * x + y * y);
    }
    const Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
    xc = -0.5 * sol(0);
    yc = -0.5 * sol(1);
    const double r2 = xc * xc + yc * yc - sol(2);
    if (!(r2 > 0.12 * 0.12 && r2 < 0.45 * 0.45)) return false;
    if (std::hypot(xc, yc) < 0.3) return false;
    radius = std::sqrt(r2);

    double err = 0.0;
    double max_chord = 0.0;
    for (int i = 0; i < n; ++i) {
        const auto& s = plate_arc_[static_cast<size_t>(i)];
        err += std::abs(std::hypot(s.x - xc, s.y - yc) - radius);
        for (int j = i + 1; j < n; j += std::max(1, n / 8)) {
            const auto& t = plate_arc_[static_cast<size_t>(j)];
            max_chord = std::max(max_chord, std::hypot(s.x - t.x, s.y - t.y));
        }
    }
    err /= static_cast<double>(n);
    // 残差过大或弧长不够 → 仍是直线/噪声，不当圆心
    return err <= 0.05 && max_chord >= 0.18;
}

bool Tracker::center_motion_is_slow() const {
    auto slow_from_hist = [](const std::deque<XySample>& hist) -> std::optional<bool> {
        if (hist.size() < 2) return std::nullopt;
        const double newest = hist.back().t;
        const XySample* old = nullptr;
        for (auto it = hist.rbegin(); it != hist.rend(); ++it) {
            if (newest - it->t >= 0.4) {
                old = &(*it);
                break;
            }
        }
        if (old == nullptr) return std::nullopt;
        // 0.28 m / 0.4 s：原地板噪声/换板伪影常到 0.15–0.25，过紧会误切正前瞄。
        return std::hypot(hist.back().x - old->x, hist.back().y - old->y) < 0.28;
    };

    // 双板心优先；只有拟合心时也要用 fit_hist，否则永远判原地、平移正前永远进不去。
    if (auto s = slow_from_hist(obs_center_hist_)) return *s;
    if (spin_center_converged_) {
        if (auto s = slow_from_hist(fit_hist_)) return *s;
    }
    return true;
}

bool Tracker::center_from_one_armor(const Armor& armor, double r, double& xc, double& yc) const {
    if (r < 0.05) return false;
    double yaw = armor.yaw;
    const bool has_prev = spin_center_valid_ || (std::hypot(target_state_(0), target_state_(2)) > 0.3);
    const double prev_x = spin_center_valid_ ? spin_center_x_ : target_state_(0);
    const double prev_y = spin_center_valid_ ? spin_center_y_ : target_state_(2);
    resolve_center_from_armor(armor.pos(0), armor.pos(1), r, yaw, xc, yc, has_prev, prev_x, prev_y);
    return std::hypot(xc, yc) > 0.3;
}

void Tracker::hold_center_when_stationary(const std::vector<Armor>& armors) {
    if (!spin_) {
        last_dual_innov_ = -1.0;
        return;
    }

    double dual_x = 0.0, dual_y = 0.0;
    bool have_dual = false;
    last_dual_innov_ = -1.0;
    {
        std::vector<std::pair<double, double>> centers;
        const double r1 = std::max(target_state_(8), 0.12);
        const double r2 = std::max(another_r_, 0.12);
        for (const auto& a : armors) {
            if (a.id != tracked_id) continue;
            double xc1 = 0.0, yc1 = 0.0, xc2 = 0.0, yc2 = 0.0;
            const bool ok1 = center_from_one_armor(a, r1, xc1, yc1);
            const bool ok2 = center_from_one_armor(a, r2, xc2, yc2);
            if (!ok1 && !ok2) continue;
            const double ref_x = spin_center_valid_ ? spin_center_x_ : target_state_(0);
            const double ref_y = spin_center_valid_ ? spin_center_y_ : target_state_(2);
            if (ok1 && ok2) {
                const double d1 = std::hypot(xc1 - ref_x, yc1 - ref_y);
                const double d2 = std::hypot(xc2 - ref_x, yc2 - ref_y);
                if (d2 + 0.03 < d1) centers.emplace_back(xc2, yc2);
                else centers.emplace_back(xc1, yc1);
            } else if (ok1) {
                centers.emplace_back(xc1, yc1);
            } else {
                centers.emplace_back(xc2, yc2);
            }
        }
        if (centers.size() >= 2) {
            double sx = 0.0, sy = 0.0;
            for (const auto& c : centers) {
                sx += c.first;
                sy += c.second;
            }
            dual_x = sx / static_cast<double>(centers.size());
            dual_y = sy / static_cast<double>(centers.size());
            have_dual = true;
            last_dual_innov_ = std::hypot(dual_x - target_state_(0), dual_y - target_state_(2));
            obs_center_hist_.push_back({move_clock_, dual_x, dual_y});
            while (!obs_center_hist_.empty() && move_clock_ - obs_center_hist_.front().t > 0.6) {
                obs_center_hist_.pop_front();
            }
        }
    }

    double fit_x = 0.0, fit_y = 0.0, fit_r = 0.0;
    const bool have_fit = fit_center_from_plate_arc(fit_x, fit_y, fit_r);

    double raw_x = 0.0, raw_y = 0.0;
    bool have_raw = false;
    double sigma = 0.07;
    // 轨迹拟合不依赖 yaw 二义；双板仍走单板几何反推，和拟合差太大时以拟合为准。
    if (have_fit && have_dual) {
        const double disagree = std::hypot(dual_x - fit_x, dual_y - fit_y);
        if (disagree > 0.20) {
            have_dual = false;
        }
    }
    if (have_dual) {
        raw_x = dual_x;
        raw_y = dual_y;
        have_raw = true;
        sigma = 0.035;
    } else if (have_fit) {
        raw_x = fit_x;
        raw_y = fit_y;
        have_raw = true;
        sigma = 0.05;
        (void)fit_r;  // 半径仍用车型表/EKF；弧拟合 r 易偏小，不写回
    }

    if (have_raw) {
        const double x_before = target_state_(0);
        const double y_before = target_state_(2);
        const double innov = std::hypot(raw_x - x_before, raw_y - y_before);
        // 拟合/双板只当弱观测。单帧不许把车心钉到几何解上，每帧最多挪 2 cm。
        if (innov < 0.6) {
            ekf_.setState(target_state_);
            if (ekf_.update_center_xy(raw_x, raw_y, std::max(sigma, 0.12))) {
                target_state_ = ekf_.getState();
            }
            double dx = target_state_(0) - x_before;
            double dy = target_state_(2) - y_before;
            const double step = std::hypot(dx, dy);
            constexpr double max_step = 0.02;
            if (step > max_step) {
                target_state_(0) = x_before + dx * max_step / step;
                target_state_(2) = y_before + dy * max_step / step;
            }
            spin_center_x_ = target_state_(0);
            spin_center_y_ = target_state_(2);
            spin_center_valid_ = true;
            spin_center_converged_ = true;
            last_center_accept_clock_ = move_clock_;
            fit_hist_.push_back({move_clock_, spin_center_x_, spin_center_y_});
            while (!fit_hist_.empty() && move_clock_ - fit_hist_.front().t > 0.5) {
                fit_hist_.pop_front();
            }
        }
    } else if (spin_center_converged_ && last_center_accept_clock_ >= 0.0 &&
               move_clock_ - last_center_accept_clock_ > 1.2) {
        // 长时间无双板/拟合更新：宁可不信旧心，回到跟板，避免平移后还钉着过期圆心。
        spin_center_converged_ = false;
        spin_center_valid_ = false;
        spin_aim_translate_ = false;
        spin_aim_mode_clock_ = 0.0;
    }

    const bool translating = spin_center_converged_ && !center_motion_is_slow();
    if (!translating) {
        target_state_(1) = 0.0;
        target_state_(3) = 0.0;
        target_state_(5) = 0.0;
        ekf_.setState(target_state_);
        return;
    }

    if (fit_hist_.size() >= 2) {
        const double span = fit_hist_.back().t - fit_hist_.front().t;
        const double moved = std::hypot(fit_hist_.back().x - fit_hist_.front().x,
                                        fit_hist_.back().y - fit_hist_.front().y);
        if (span >= 0.4 && moved >= 0.20) {
            target_state_(1) = (fit_hist_.back().x - fit_hist_.front().x) / span;
            target_state_(3) = (fit_hist_.back().y - fit_hist_.front().y) / span;
            target_state_(5) = 0.0;
        }
    }
    ekf_.setState(target_state_);
}

void Tracker::update_spin_aim_mode(double dt) {
    if (!spin_) {
        spin_aim_translate_ = false;
        spin_aim_mode_clock_ = 0.0;
        return;
    }
    // 车心未收敛前绝不切正前；无双板/拟合时 center_motion_is_slow==true → 跟板。
    const bool want_trans = spin_center_converged_ && !center_motion_is_slow();
    if (want_trans == spin_aim_translate_) {
        spin_aim_mode_clock_ = 0.0;
        return;
    }
    spin_aim_mode_clock_ += dt;
    const double need = want_trans ? kAimEnterTrans : kAimLeaveTrans;
    if (spin_aim_mode_clock_ >= need) {
        spin_aim_translate_ = want_trans;
        spin_aim_mode_clock_ = 0.0;
    }
}

std::pair<int, double> Tracker::select_spin_front_plate(double front_yaw, double yaw_arr, double v_yaw) {
    auto facing_err = [&](int i, double yaw_ref) {
        const double armor_yaw = yaw_ref - i * (M_PI / 2.0);
        return std::abs(shortest_angular_distance(front_yaw, armor_yaw));
    };

    const double abs_w = std::max(std::abs(v_yaw), 1e-3);
    const double switch_adv = std::min(kSwitchAdvMax, kSwitchThrRad / abs_w);
    const double yaw_sw = yaw_arr + v_yaw * switch_adv;

    int best = spin_front_plate_i_;
    if (best < 0 || best > 3) best = 0;
    for (int i = 0; i < 4; ++i) {
        if (facing_err(i, yaw_sw) + kSwitchThrRad < facing_err(best, yaw_sw)) {
            best = i;
        }
    }

    if (std::abs(v_yaw) > 0.3) {
        const double armor_yaw = yaw_arr - best * (M_PI / 2.0);
        const double signed_err = shortest_angular_distance(front_yaw, armor_yaw);
        const double leave_err = std::abs(shortest_angular_distance(
            front_yaw, armor_yaw - std::copysign(kLeaveBiasRad, v_yaw)));
        if (leave_err < std::abs(signed_err) && signed_err * v_yaw > 0.0) {
            best = (v_yaw > 0.0) ? (best + 1) % 4 : (best + 3) % 4;
        }
    }

    spin_front_plate_i_ = best;
    const double armor_yaw_arr = yaw_arr - best * (M_PI / 2.0);
    const double plate_err = std::abs(shortest_angular_distance(front_yaw, armor_yaw_arr));
    return {best, plate_err};
}

void Tracker::apply_spin_period() {
    // 非 SPIN 不碰 v_yaw，交给 EKF。
    if (!spin_) return;
    if (spin_periods_.empty() || last_jump_sign_ == 0) {
        // 周期窗空档（变速、短暂丢板）：沿用上次有效转速，别让提前量/换板突然归零。
        if (std::abs(last_spin_vyaw_) > 1e-3) {
            target_state_(7) = last_spin_vyaw_;
        }
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
    last_spin_vyaw_ = target_state_(7);
}

bool Tracker::spin_period_is_stable() const {
    if (spin_periods_.size() < 3) return false;
    double sum = 0.0;
    double sum2 = 0.0;
    for (double period : spin_periods_) {
        sum += period;
        sum2 += period * period;
    }
    const double n = static_cast<double>(spin_periods_.size());
    const double mean = sum / n;
    if (mean < 1e-3) return false;
    const double var = std::max(0.0, sum2 / n - mean * mean);
    return std::sqrt(var) / mean < 0.15;
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

    // 小陀螺换板时下一块板离预测点常超过 0.8 m，同一编号仍算这块车。
    const double match_limit = spin_ ? 1.2 : kSameRobot;
    if (!match.has_value() || best > match_limit) {
        // 撤销本帧外推。换板间隙若继续按切向速度走，瞄准点会飞出画面，SPIN 标志却还在。
        plate_(0) -= plate_(1) * dt;
        plate_(2) -= plate_(3) * dt;
        plate_(4) -= plate_(5) * dt;
        plate_(1) = 0.0;
        plate_(3) = 0.0;
        plate_(5) = 0.0;
        move_clock_ += dt;
        update_lead_velocity(dt);
        plate_move_speed_ = 0.0;
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
    // 板观测的切向残差会灌进车心，是 SPIN 下车心飘的主因之一。
    const double xc_before_armor = target_state_(0);
    const double yc_before_armor = target_state_(2);

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

    // 装甲观测的切向残差会把车心一帧拉开。SPIN 时这一步不改 xc/yc，
    // 车心只由后面的拟合/双板弱观测更新，并且每帧最多 2 cm。
    if (spin_ && ekf_matched) {
        target_state_(0) = xc_before_armor;
        target_state_(2) = yc_before_armor;
        if (center_motion_is_slow()) {
            target_state_(1) = 0.0;
            target_state_(3) = 0.0;
        }
        ekf_.setState(target_state_);
    }

    const bool radius_railed = target_state_(8) < radius_params.r_min || target_state_(8) > radius_params.r_max;
    // 钳制状态范围防发散
    target_state_(8) = std::clamp(target_state_(8), radius_params.r_min, radius_params.r_max);
    target_state_(1) = std::clamp(target_state_(1), -15.0, 15.0);
    target_state_(3) = std::clamp(target_state_(3), -15.0, 15.0);
    target_state_(5) = std::clamp(target_state_(5), -2.0, 2.0);

    // 车心跟不上这块板时丢掉 9 维状态，用当前板重新初始化。板自己的速度留着继续瞄。
    const bool plate_ok = plate_meas_.has_value();
    bool did_ekf_reset = false;
    if (plate_ok && (!ekf_matched || radius_railed)) {
        if (++ekf_reject_streak_ >= 5) {
            // 连续拒绝会整状态重开，转速被写成 0。小陀螺还在转，这一下不是停转。
            const double kept_vyaw = spin_ ? target_state_(7) : 0.0;
            const double kept_x = target_state_(0);
            const double kept_y = target_state_(2);
            const double kept_vx = spin_ ? target_state_(1) : 0.0;
            const double kept_vy = spin_ ? target_state_(3) : 0.0;
            init_ekf(*plate_meas_);
            if (spin_) {
                // 平移小陀螺时清 vx/vy 会让正前点下一帧跳回，枪口跟着撕。
                target_state_(0) = kept_x;
                target_state_(2) = kept_y;
                target_state_(1) = kept_vx;
                target_state_(3) = kept_vy;
                target_state_(7) = kept_vyaw;
            }
            ekf_reject_streak_ = 0;
            did_ekf_reset = true;
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
    if (plate_meas_.has_value()) {
        note_plate_arc(plate_meas_->pos(0), plate_meas_->pos(1));
    }
    hold_center_when_stationary(armors);
    apply_spin_period();
    ekf_.setState(target_state_);

    // bag 扫参：RM_TRACKER_METRICS=/path.csv 时逐帧写出 spin 段指标。
    if (const char* metrics_path = std::getenv("RM_TRACKER_METRICS")) {
        static FILE* metrics_file = nullptr;
        static bool metrics_hdr = false;
        if (!metrics_file) {
            metrics_file = std::fopen(metrics_path, "w");
            metrics_hdr = false;
        }
        if (metrics_file) {
            if (!metrics_hdr) {
                std::fprintf(metrics_file, "t,spin,xc,yc,matched,reset,dual_innov,converged,period_stable\n");
                metrics_hdr = true;
            }
            std::fprintf(metrics_file, "%.4f,%d,%.5f,%.5f,%d,%d,%.5f,%d,%d\n",
                         move_clock_, spin_ ? 1 : 0, target_state_(0), target_state_(2),
                         ekf_matched ? 1 : 0, did_ekf_reset ? 1 : 0, last_dual_innov_,
                         spin_center_converged_ ? 1 : 0,
                         spin_period_is_stable() ? 1 : 0);
            std::fflush(metrics_file);
        }
    }

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

std::optional<Armor> Tracker::world_to_muzzle(RmTF& tf, const Armor& target, const Eigen::Vector3d& offset_pos, const Eigen::Vector3d& imu_rpy) {
    Eigen::Vector3d pos_target_world = target.pos;
    Eigen::Vector3d offset_pos_world = tf.cam_to_world(offset_pos, imu_rpy);
    Eigen::Vector3d pos_gun_frame_world = pos_target_world - offset_pos_world;

    Armor muzzle_target(target.id, pos_gun_frame_world(0), pos_gun_frame_world(1), pos_gun_frame_world(2), target.yaw, target.index);
    muzzle_target.angle_diff = target.angle_diff;
    return muzzle_target;
}

bool Tracker::yaw_in_armor_edge_window(double x, double y, double armor_yaw, double armor_w,
                                       double delta_yaw_rad, double shrink,
                                       double* allow_min_rad, double* allow_max_rad) const {
    // 装甲左右端点（与 FaterYU choose_armor 同式），角窗相对板心视线。
    const double ax = x - 0.5 * armor_w * std::sin(armor_yaw);
    const double ay = y + 0.5 * armor_w * std::cos(armor_yaw);
    const double bx = x + 0.5 * armor_w * std::sin(armor_yaw);
    const double by = y - 0.5 * armor_w * std::cos(armor_yaw);
    const double ang_c = std::atan2(y, x);
    const double ang_a = std::atan2(ay, ax);
    const double ang_b = std::atan2(by, bx);
    double allow_max = normalize_angle(ang_c - ang_b);
    double allow_min = normalize_angle(ang_c - ang_a);
    if (allow_min > allow_max) {
        std::swap(allow_min, allow_max);
    }
    allow_min *= shrink;
    allow_max *= shrink;
    if (allow_min_rad) *allow_min_rad = allow_min;
    if (allow_max_rad) *allow_max_rad = allow_max;
    return delta_yaw_rad > allow_min && delta_yaw_rad < allow_max;
}

// =====================================================================
// 5. 动态开火门限：装甲边缘角窗（FaterYU）+ 小陀螺正对弧长
// =====================================================================
std::tuple<double, double, bool> Tracker::can_fire(const Armor& target, std::tuple<double, double, bool> gimbal_control) {
    double yaw = std::get<0>(gimbal_control);
    double pitch = std::get<1>(gimbal_control);
    bool is_valid = std::get<2>(gimbal_control);

    if (!is_valid) return {yaw, pitch, false};

    double armor_w = 0.135, armor_h = 0.125;
    for (const auto& r : robot_list_) {
        if (r.id == target.id) {
            armor_w = r.armor_width / 1000.0;
            armor_h = r.armor_height / 1000.0;
            break;
        }
    }

    constexpr double kEdgeShrink = 0.85;
    double allow_min = 0.0, allow_max = 0.0;
    const double dyaw_rad = yaw * DEG2RAD;
    const bool in_edge = yaw_in_armor_edge_window(
        target.pos(0), target.pos(1), target.yaw, armor_w, dyaw_rad, kEdgeShrink,
        &allow_min, &allow_max);

    // HUD：用真实角窗半宽，比 atan(w/2/dist) 更贴斜对时的投影。
    const double half_span_deg = 0.5 * (std::abs(allow_min) + std::abs(allow_max)) * RAD2DEG;
    double dynamic_pitch_tol = std::atan((armor_h / 2.0) / std::max(target.dist, 0.1)) * RAD2DEG * 0.8;
    double yaw_tol_mix = std::clamp(half_span_deg, 1.0, 8.0);
    double pitch_tol_mix = std::clamp(dynamic_pitch_tol, 1.0, 5.0);
    this->yaw_tolerance_deg = yaw_tol_mix;
    this->pitch_tolerance_deg = pitch_tol_mix;

    bool fire_flag = false;
    if (spin_) {
        // 板还在正对窗口（弧长）且枪口误差落在装甲左右边缘角内。
        const double half_w = armor_w * 0.5 - 0.0085;
        fire_flag = (in_edge &&
                     std::abs(pitch) < pitch_tol_mix * 0.8 &&
                     target.dist <= shootable_dist &&
                     target.target_to_armor_dist < half_w);
    } else {
        fire_flag = (in_edge &&
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

    if (!plate_inited_) {
        return {gimbal_control, logs};
    }
    update_spin_aim_mode(dt);

    Armor target_cam(tracked_id, 0.0, 0.0, 0.0, last_plate_yaw_);
    bool aimed = false;
    const double center_norm = std::hypot(target_state_(0), target_state_(2));

    // 换板间隔稳定才预测下一块板。飞行时间内转过超过 75° 就不再预测：
    // 那会跨过一整块板，圆心稍偏红圈就离开车身。退回下面，打眼前这块板。
    const double rough_spin = std::max(0.1, center_norm - target_state_(8));
    const double t_flight_spin = (rough_spin <= LUT_DIST_MAX)
                                 ? lut_lookup_tof(rough_spin, target_state_(4))
                                 : rough_spin / std::max(bullet_speed, 1e-3);
    const double spin_lead = std::abs(target_state_(7)) * (t_flight_spin + system_delay);
    const bool spin_lead_ok = spin_lead < 75.0 * DEG2RAD;
    if (spin_ && spin_period_is_stable() && spin_lead_ok && center_norm > 0.3) {
        const double x0 = target_state_(0), y0 = target_state_(2), z0 = target_state_(4);
        const double vx = spin_aim_translate_ ? target_state_(1) : 0.0;
        const double vy = spin_aim_translate_ ? target_state_(3) : 0.0;
        const double vz = spin_aim_translate_ ? target_state_(5) : 0.0;
        const double t_flight = t_flight_spin;
        const double select_t = t_flight + system_delay;
        const double aim_t = select_t + std::max(0.0, controller_delay_);
        const double xc_s = x0 + vx * select_t;
        const double yc_s = y0 + vy * select_t;
        const double yaw_arr = target_state_(6) + target_state_(7) * select_t;
        const double select_norm = std::hypot(xc_s, yc_s);
        if (select_norm > 0.3) {
            const double front_yaw_s = std::atan2(yc_s, xc_s);
            const auto [plate_i, plate_err] =
                select_spin_front_plate(front_yaw_s, yaw_arr, target_state_(7));
            const bool even = (plate_i % 2 == 0);
            const double r = even ? target_state_(8) : another_r_;
            const double xc_a = x0 + vx * aim_t;
            const double yc_a = y0 + vy * aim_t;
            const double za_a = z0 + vz * aim_t;
            const double aim_norm = std::hypot(xc_a, yc_a);
            if (aim_norm > 0.3) {
                const double front_yaw_a = std::atan2(yc_a, xc_a);
                const double z = even ? za_a : (za_a + dz_);
                const double ux = xc_a / aim_norm, uy = yc_a / aim_norm;
                target_cam = Armor(tracked_id, xc_a - r * ux, yc_a - r * uy, z, front_yaw_a, plate_i);
                target_cam.target_to_armor_dist = r * plate_err;
                target_cam.angle_diff = plate_err;
                target_cam.dist = std::hypot(target_cam.pos(0), target_cam.pos(1), target_cam.pos(2));
                const Eigen::Vector3d plate_now(plate_(0), plate_(2), plate_(4));
                // 预测点离开当前观测板超过半个车身，当作圆心偏了，不采用。
                if ((target_cam.pos - plate_now).norm() < 0.45) {
                    aimed = true;
                }
            }
        }
    }

    // 非 SPIN / 原地 SPIN / 正前失败：跟检测板。SPIN 原地不加切向 lead。
    if (!aimed) {
        const Eigen::Vector3d plate_pos(plate_(0), plate_(2), plate_(4));
        const double dist_h = std::hypot(plate_pos(0), plate_pos(1));
        const double t_flight = (dist_h <= LUT_DIST_MAX)
                                ? lut_lookup_tof(dist_h, plate_pos(2))
                                : dist_h / std::max(bullet_speed, 1e-3);
        const double lead_t = t_flight + system_delay + std::max(0.0, controller_delay_);
        const Eigen::Vector3d aim = spin_ ? plate_pos : (plate_pos + lead_vel_ * lead_t);
        target_cam = Armor(tracked_id, aim(0), aim(1), aim(2), last_plate_yaw_);
        target_cam.dist = std::hypot(aim(0), aim(1), aim(2));
        if (spin_) {
            const double los = std::atan2(plate_pos(1), plate_pos(0));
            const double face_err = std::abs(shortest_angular_distance(los, last_plate_yaw_));
            const double r = std::max(target_state_(8), 0.2);
            target_cam.target_to_armor_dist = r * face_err;
            target_cam.angle_diff = face_err;
        } else {
            target_cam.target_to_armor_dist = 0.0;
        }
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
