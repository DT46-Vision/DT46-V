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

    build_ballistic_lut();
}

void Tracker::try_init_tracker(std::vector<Armor>& armors) {
    if (armors.empty()) return;

    // Inputs are already in stabilized axes; Z is height, not camera depth.
    std::sort(armors.begin(), armors.end(), [](const Armor& a, const Armor& b) {
        return a.dist < b.dist;
    });

    Armor chosen_armor = armors[0];

    lock_target(chosen_armor);
}

void Tracker::init_ekf(const Armor& armor) {
    double xa = armor.pos(0), ya = armor.pos(1), za = armor.pos(2);
    const double r_init = (radius_params.r_min + radius_params.r_max) * .5;
    const double yaw = armor.yaw;
    BodyEkf::State body=BodyEkf::State::Zero();
    body[0]=xa+r_init*std::cos(yaw);body[2]=ya+r_init*std::sin(yaw);
    body[4]=za;body[6]=yaw;body[8]=body[9]=r_init;
    target_state_=body.head<9>();another_r_=r_init;dz_=0;spin_=false;
    // The first board is measured; its inferred center and radius are correlated.
    // Propagate independent (board xyz, yaw, rA, rB, hB, velocities) priors through
    // c = board + rA * normal. Independent center/radius priors invent board error.
    const double sigma=.005+ekf_QR_params.r_xyz_factor*.01*armor.dist;
    const double yaw_scale=std::max(1.0,armor.dist/std::max(.1,ekf_QR_params.stable_dist));
    BodyEkf::Covariance prior=BodyEkf::Covariance::Zero();
    const double depth_sigma=ekf_QR_params.r_depth_factor>0?.005+ekf_QR_params.r_depth_factor*.01*armor.dist*armor.dist:0;
    const Eigen::Matrix3d position_covariance=BodyEkf::measurement_covariance(armor.pos,sigma,1,depth_sigma).topLeftCorner<3,3>();
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)prior(2*i,2*j)=position_covariance(i,j);
    prior(6,6)=std::clamp(ekf_QR_params.r_yaw*yaw_scale*yaw_scale,1e-6,10.0);
    prior(1,1)=prior(3,3)=prior(5,5)=1.;prior(7,7)=50.;
    prior(8,8)=prior(9,9)=prior(10,10)=.01;
    BodyEkf::Covariance transform=BodyEkf::Covariance::Identity();
    transform(0,6)=-r_init*std::sin(yaw);transform(2,6)=r_init*std::cos(yaw);
    transform(0,8)=std::cos(yaw);transform(2,8)=std::sin(yaw);
    const BodyEkf::Covariance covariance=transform*prior*transform.transpose();
    body_ekf_.init(body,covariance);body_plate_index_=0;selected_body_plate_=-1;body_matched_=false;detect_elapsed_s_=lost_elapsed_s_=0;
    body_ekf_.set_limits(radius_params.r_min,radius_params.r_max,max_match_distance,max_match_yaw_diff);
    body_spin_mode_=false;body_spin_enter_s_=body_spin_exit_s_=0;
}
void Tracker::lock_target(const Armor& armor) {
    tracked_id = armor.id;
    init_ekf(armor);
    tracker_state = TrackerState::DETECTING;
}

Eigen::Vector3d Tracker::plate_position() const {
    return body_ekf_.plate(body_plate_index_).head<3>();
}

Eigen::Vector3d Tracker::plate_velocity() const {
    const auto& state=body_ekf_.state();
    const double yaw=state[6]+body_plate_index_*M_PI/2;
    const double speed=state[8+body_plate_index_%2]*state[7];
    return {state[1]+speed*std::sin(yaw),state[3]-speed*std::cos(yaw),state[5]};
}

void Tracker::update(const std::vector<Armor>& armors, double dt, std::vector<std::pair<std::string, std::string>>& logs) {
    body_ekf_.predict(dt,ekf_QR_params.q_xyz,ekf_QR_params.q_yaw,ekf_QR_params.q_r);
    std::vector<BodyEkf::Observation> observations;
    for(const auto& armor:armors)if(armor.id==tracked_id)observations.push_back({armor.pos,armor.yaw});
    const double distance=std::hypot(target_state_(0),target_state_(2));
    const double sigma=.005+ekf_QR_params.r_xyz_factor*.01*distance;
    const double yaw_scale=std::max(1.0,distance/std::max(.1,ekf_QR_params.stable_dist));
    const double depth_sigma=ekf_QR_params.r_depth_factor>0?.005+ekf_QR_params.r_depth_factor*.01*distance*distance:0;
    const double yaw_variance=std::clamp(ekf_QR_params.r_yaw*yaw_scale*yaw_scale,1e-6,10.0);
    const int matched=body_ekf_.update(observations,sigma,yaw_variance,depth_sigma);
    body_matched_=matched>=0;
    if(body_matched_)body_plate_index_=matched;
    const auto& state=body_ekf_.state();
    if(body_matched_){
        if(std::abs(state[7])>=min_spinning_vel_){body_spin_enter_s_+=dt;body_spin_exit_s_=0;}
        else {body_spin_exit_s_+=dt;body_spin_enter_s_=0;}
        if(body_spin_enter_s_>=spin_enter_time_)body_spin_mode_=true;
        if(body_spin_exit_s_>=spin_exit_time_)body_spin_mode_=false;
    }
    spin_=body_spin_mode_;
    const int group=body_plate_index_%2;
    target_state_.head<8>()=state.head<8>();
    target_state_(4)=state[4]+group*state[10];
    target_state_(6)=normalize_angle(state[6]+body_plate_index_*M_PI/2);
    target_state_(8)=state[8+group];another_r_=state[9-group];
    dz_=group?-state[10]:state[10];
    const bool plate_ok=body_matched_;
    logs.emplace_back("body_nis",std::to_string(body_ekf_.last_nis()));
    logs.emplace_back("body_association",body_ekf_.rejection_reason());
    // Keep existing frame-count configuration as nominal 10 ms durations, independent of delivery rate.
    if(plate_ok){
        lost_elapsed_s_=0;detect_elapsed_s_+=dt;
        if(tracker_state==TrackerState::TEMP_LOST || detect_elapsed_s_>=tracking_thres*.01)
            tracker_state=TrackerState::TRACKING;
    }else{
        if(tracker_state==TrackerState::DETECTING)detect_elapsed_s_=0;
        lost_elapsed_s_+=dt;
        if(lost_elapsed_s_>lost_thres*.01)tracker_state=TrackerState::LOST;
        else if(tracker_state==TrackerState::TRACKING)tracker_state=TrackerState::TEMP_LOST;
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
// 5. 动态开火门限：装甲投影角窗与枪口误差
// =====================================================================
std::tuple<double, double, bool> Tracker::can_fire(const Armor& target, std::tuple<double, double, bool> gimbal_control) {
    double yaw = std::get<0>(gimbal_control);
    double pitch = std::get<1>(gimbal_control);
    bool is_valid = std::get<2>(gimbal_control);

    if (!is_valid) return {yaw, pitch, false};

    constexpr double armor_w = .135, armor_h = .125;

    constexpr double kEdgeShrink = 0.85;
    double allow_min = 0.0, allow_max = 0.0;
    const double dyaw_rad = yaw * DEG2RAD;
    const bool in_edge = yaw_in_armor_edge_window(
        target.pos(0), target.pos(1), target.yaw, armor_w, dyaw_rad, kEdgeShrink,
        &allow_min, &allow_max);

    // HUD：用真实角窗半宽，比 atan(w/2/dist) 更贴斜对时的投影。
    const double half_span_deg = 0.5 * (std::abs(allow_min) + std::abs(allow_max)) * RAD2DEG;
    double dynamic_pitch_tol = std::atan((armor_h / 2.0) / std::max(target.dist, 0.1)) * RAD2DEG * 0.8;
    const double yaw_tol_mix=std::min(half_span_deg,yaw_tolerance_deg);
    const double pitch_tol_mix=std::min(dynamic_pitch_tol,pitch_tolerance_deg);
    fire_yaw_tolerance_deg_=yaw_tol_mix;fire_pitch_tolerance_deg_=pitch_tol_mix;

    const double facing=std::cos(shortest_angular_distance(std::atan2(target.pos.y(),target.pos.x()),target.yaw));
    const bool fire_flag=in_edge && facing>0 && std::abs(yaw)<yaw_tolerance_deg &&
        std::abs(pitch)<pitch_tol_mix && target.dist<=shootable_dist;

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
    double di = std::clamp((dist_h - LUT_DIST_MIN) / LUT_DIST_STEP,0.0,double(LUT_DIST_BINS-1));
    double dj = std::clamp((z - LUT_HEIGHT_MIN) / LUT_HEIGHT_STEP,0.0,double(LUT_HEIGHT_BINS-1));

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
    double di = std::clamp((dist_h - LUT_DIST_MIN) / LUT_DIST_STEP,0.0,double(LUT_DIST_BINS-1));
    double dj = std::clamp((z - LUT_HEIGHT_MIN) / LUT_HEIGHT_STEP,0.0,double(LUT_HEIGHT_BINS-1));

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

    if(!std::isfinite(z)||!std::isfinite(dist_h)||dist_h>LUT_DIST_MAX||z<LUT_HEIGHT_MIN||z>LUT_HEIGHT_MAX)return {0,0,false};
    double pitch_rad = lut_lookup(dist_h, z);

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
    target_cam_cache_.reset();target_muzzle_cache_.reset();gimbal_control_cache_={0,0,false};
    fire_blockers_.clear();aim_lateral_sigma_=aim_vertical_sigma_=aim_hit_probability_=0;
    body_matched_=false;
    fire_yaw_tolerance_deg_=fire_pitch_tolerance_deg_=0;
    if(!std::isfinite(dt)||dt<0||(dt==0&&tracker_state!=TrackerState::LOST)){fire_blockers_.push_back("invalid_time");return {gimbal_control,logs};}
    if(dt>.5){tracker_state=TrackerState::LOST;body_matched_=false;}

    // 1. 判断并调度基础状态机
    if (tracker_state == TrackerState::LOST) {
        fire_blockers_.push_back("acquiring");
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
        fire_blockers_.push_back("lost");
        tracked_id = -1;
        logs.emplace_back("state", "TRACKING/TEMP_LOST -> LOST (target lost)");
        return {gimbal_control, logs};
    }

    // ==========================================================
    // 【主流程重构】
    // ==========================================================

    const auto& body=body_ekf_.state();
    const double center_range=std::hypot(body[0],body[2]);
    // The selected board is on the near half of the body. A near-surface seed
    // avoids adding an entire chassis radius to the initial flight distance.
    const double flight=lut_lookup_tof(std::max(.2,center_range-std::min(body[8],body[9])),body[4]);
    const double ahead=flight+std::max(0.0,system_delay)+std::max(0.0,controller_delay_);
    int chosen=body_plate_index_;double best_score=std::numeric_limits<double>::infinity();
    for(int i=0;i<4;++i){
        const auto predicted=body_ekf_.plate(i,ahead);
        const double face=std::abs(shortest_angular_distance(std::atan2(predicted[1],predicted[0]),predicted[3]));
        const double score=face+(i==selected_body_plate_?0:.08);
        if(score<best_score){best_score=score;chosen=i;}
    }
    selected_body_plate_=chosen;
    const auto predicted=body_ekf_.plate(chosen,ahead);
    Armor target_cam(tracked_id,predicted[0],predicted[1],predicted[2],predicted[3],chosen);
    target_cam.angle_diff=std::abs(shortest_angular_distance(std::atan2(predicted[1],predicted[0]),predicted[3]));
    // 4. 转换至枪口系并打靶
    std::optional<Armor> target_muzzle = world_to_muzzle(tf, target_cam, cam_to_gun_pos, imu_rpy);
    if (!target_muzzle.has_value()) {
        return {gimbal_control, logs};
    }

    auto [yaw_cmd_rad, pitch_cmd_rad, isValid] = solve_ballistic(tf, target_muzzle.value(), cam_to_gun_rpy, imu_rpy);

    // 5. 动态发弹许可
    // 注意：solve_ballistic 返回的是弧度，但 can_fire 期望的是角度
    auto final_gimbal = can_fire(*target_muzzle, {yaw_cmd_rad * RAD2DEG, pitch_cmd_rad * RAD2DEG, isValid});

    const double actual_flight=lut_lookup_tof(std::hypot(target_muzzle->pos[0],target_muzzle->pos[1]),target_muzzle->pos[2]);
    const auto arrival=body_ekf_.plate(chosen,actual_flight+std::max(0.0,system_delay)+std::max(0.0,controller_delay_));
    const bool prediction_consistent=(arrival.head<3>()-target_cam.pos).norm()<.025;
    const auto forecast=body_ekf_.forecast(actual_flight+std::max(0.0,system_delay)+std::max(0.0,controller_delay_));
    const Eigen::Matrix3d covariance=forecast.plate_covariance(chosen);
    const double bearing=std::atan2(target_muzzle->pos.y(),target_muzzle->pos.x());
    const Eigen::Vector3d tangent(-std::sin(bearing),std::cos(bearing),0);
    aim_lateral_sigma_=std::sqrt(std::max(0.0,tangent.dot(covariance*tangent)));
    aim_vertical_sigma_=std::sqrt(std::max(0.0,covariance(2,2)));
    // Approximate rectangular hit probability in sightline coordinates. Depth
    // uncertainty does not become lateral error just because the plate is oblique.
    const double projected_half_width=.135*.5*.85*std::max(0.0,std::cos(arrival[3]-bearing));
    const double half_height=.125*.5*.8;
    const double range=std::hypot(target_muzzle->pos.x(),target_muzzle->pos.y());
    const auto inside_probability=[](double half_size,double error,double sigma){
        const double scale=std::sqrt(2.)*std::max(sigma,1e-9);
        return .5*(std::erf((half_size-error)/scale)+std::erf((half_size+error)/scale));
    };
    aim_hit_probability_=inside_probability(projected_half_width,range*std::tan(yaw_cmd_rad),aim_lateral_sigma_)*
        inside_probability(half_height,range*std::tan(pitch_cmd_rad),aim_vertical_sigma_);
    if(!isValid)fire_blockers_.push_back("ballistic_domain");
    if(!std::get<2>(final_gimbal))fire_blockers_.push_back("alignment_or_facing");
    if(!body_matched_)fire_blockers_.push_back("unmatched");
    if(!body_ekf_.established())fire_blockers_.push_back("insufficient_updates");
    if(tracker_state!=TrackerState::TRACKING)fire_blockers_.push_back("not_tracking");
    if(!prediction_consistent)fire_blockers_.push_back("flight_consistency");
    if(!std::isfinite(aim_hit_probability_)||aim_hit_probability_<min_hit_probability)
        fire_blockers_.push_back("arrival_probability");
    std::get<2>(final_gimbal)=fire_blockers_.empty();
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
    s.body_state=body_ekf_.state();s.body_covariance=body_ekf_.covariance();s.body_ready=body_ekf_.ready();
    s.body_matched_mask=body_matched_?body_ekf_.matched_mask():0;s.observed_plate=body_matched_?body_plate_index_:-1;
    s.aim_lateral_sigma=aim_lateral_sigma_;s.aim_vertical_sigma=aim_vertical_sigma_;s.fire_blockers=fire_blockers_;s.aim_hit_probability=aim_hit_probability_;
    s.tracker_state = tracker_state;
    s.target_state = target_state_;
    s.another_r = another_r_;
    s.dz = dz_;
    s.debug_yaw_armors = debug_yaw_armors_cache_;
    s.target = target_cam_cache_;
    s.muzzle_target = target_muzzle_cache_;
    s.spin = spin_;
    s.yaw_tolerance_deg = fire_yaw_tolerance_deg_;
    s.pitch_tolerance_deg = fire_pitch_tolerance_deg_;
    s.gimbal_control = gimbal_control_cache_;
    s.ekf_yaw_vel = target_state_(7);
    s.bullet_speed = bullet_speed;
    return s;
}

} // namespace dt46_vision
