#include "aim.h"
#include "rm_detector/detector.hpp"
#include "rm_detector/pnp.hpp"
#include "rm_tracker/tracker.hpp"
#include "rm_tracker/frame_clock.hpp"
#include <algorithm>
#include <memory>
#include <stdexcept>

namespace {
thread_local std::string last_error;

double parameter(const cv::FileNode& node, const char* key, double fallback) {
    return node[key].empty() ? fallback : static_cast<double>(node[key]);
}

void configure_tracker(dt46_vision::Tracker& tracker, const cv::FileNode& config) {
#define PARAM(member, name) tracker.member = parameter(config, name, tracker.member)
    PARAM(max_match_distance, "max_match_distance");
    tracker.max_match_yaw_diff = parameter(config, "max_match_yaw_diff", 57.0) * dt46_vision::DEG2RAD;
    PARAM(jump_cooldown_max, "jump_cooldown_max");
    PARAM(tracking_thres, "tracking_thres");
    PARAM(lost_thres, "lost_thres");
    PARAM(dist_tol, "dist_tol");
    PARAM(ekf_QR_params.q_xyz, "ekf_QR_q_xyz");
    PARAM(ekf_QR_params.q_yaw, "ekf_QR_q_yaw");
    PARAM(ekf_QR_params.q_r, "ekf_QR_q_r");
    PARAM(ekf_QR_params.r_xyz_factor, "ekf_QR_r_xyz_factor");
    PARAM(ekf_QR_params.r_depth_factor, "ekf_QR_r_depth_factor");
    PARAM(ekf_QR_params.r_yaw, "ekf_QR_r_yaw");
    PARAM(ekf_QR_params.stable_dist, "ekf_QR_stable_dist");
    PARAM(radius_params.r_min, "radius_r_min");
    PARAM(radius_params.r_max, "radius_r_max");
    PARAM(system_delay, "system_delay");
    PARAM(controller_delay_, "controller_delay");
    PARAM(bullet_speed, "bullet_speed");
    PARAM(k_v2, "k_v2");
    PARAM(shootable_dist, "shootable_dist");
    PARAM(yaw_tolerance_deg, "yaw_tolerance_deg");
    PARAM(pitch_tolerance_deg, "pitch_tolerance_deg");
    PARAM(min_hit_probability, "min_hit_probability");
    PARAM(min_spinning_vel_, "min_spinning_vel");
    PARAM(spin_enter_time_, "spin_enter_time");
    PARAM(spin_exit_time_, "spin_exit_time");
#undef PARAM
    if (parameter(config, "gravity", 9.81) != 9.81)
        throw std::invalid_argument("Original tracker requires gravity = 9.81 m/s^2");
    tracker.cam_to_gun_rpy << parameter(config, "cam_to_gun_rpy_r", 0.0),
                              parameter(config, "cam_to_gun_rpy_p", 0.0),
                              parameter(config, "cam_to_gun_rpy_y", 0.0);
    tracker.build_ballistic_lut();
}

struct Pipeline {
    std::unique_ptr<DT46_VISION::ArmorDetector> detector;
    DT46_VISION::PNP pnp;
    DT46_VISION::CameraCalibration calibration{};
    std::optional<double> armor_tilt_deg;
    dt46_vision::Tracker tracker;
    dt46_vision::RmTF transform;
    double previous_time = -1.0;
    dt46_vision::FrameClock frame_clock;
    double frame_interval_s = 0;
    bool frame_reinitialized = true;
    int previous_color = -1;
    int detect_color = -1;
    double smooth_yaw = 0.0, smooth_pitch = 0.0;
    bool smooth_ready = false;
    cv::Mat bgr;
    std::string snapshot = "{}";
    std::vector<Eigen::Vector3d> observed_normals_camera;
    std::vector<DT46_VISION::PNP::Pose> observed_poses;

    Pipeline(const char* model, const char* parameters) {
        cv::setNumThreads(1);
        cv::FileStorage config(parameters, cv::FileStorage::READ | cv::FileStorage::MEMORY |
                                           cv::FileStorage::FORMAT_JSON);
        const auto source = config["detector"];
        DT46_VISION::Params p{};
#define PARAM(member, fallback) p.member = parameter(source, #member, fallback)
        PARAM(light_area_min, 5); PARAM(light_h_w_ratio, 2.0);
        PARAM(light_angle_min, -35); PARAM(light_angle_max, 35);
        PARAM(light_red_ratio, 1.0); PARAM(light_blue_ratio, 1.0);
        PARAM(height_rate_tol, 1.8);
        PARAM(height_multiplier_min, 1.0); PARAM(height_multiplier_max, 3.0);
        PARAM(row_stretch, 2.22); PARAM(col_stretch, 0.7);
        PARAM(roi_crop, 0); PARAM(roi_scale, 0.4);
#undef PARAM
        detector = std::make_unique<DT46_VISION::ArmorDetector>(
            2, parameter(source, "display_mode", 0), parameter(source, "binary_val", 120), p);
        detector->set_classifier(std::make_shared<DT46_VISION::NumberClassifier>(model));
        detect_color = int(parameter(source,"detect_color",-1));
        configure_tracker(tracker, config["tracker"]);
        if(!source["pnp_armor_tilt_deg"].empty())armor_tilt_deg=double(source["pnp_armor_tilt_deg"]);
        pnp.setArmorGeometry(parameter(source,"pnp_light_spacing_mm",135.),parameter(source,"pnp_light_height_mm",58.));
    }
};

void observe(Pipeline& pipeline, AimResult& result, const cv::Mat& frame,
             const Eigen::Matrix3d& rotation, std::vector<dt46_vision::Armor>& armors) {
    pipeline.pnp.beginFrame();
    if(pipeline.armor_tilt_deg){
        cv::Matx33d optical;for(int i=0;i<3;++i)for(int j=0;j<3;++j)optical(i,j)=rotation(i,j);
        pipeline.pnp.setUprightConstraint(optical,*pipeline.armor_tilt_deg);
    }
    pipeline.observed_normals_camera.clear();
    pipeline.observed_poses.clear();
    pipeline.detector->img_drawn.release();
    pipeline.detector->img_armor.release();
    pipeline.detector->img_armor_processed.release();
    const auto detected = pipeline.detector->detect_armors(frame);
    if (detected.size() > 64) throw std::runtime_error("More than 64 armor candidates");
    result.light_count = static_cast<int>(pipeline.detector->lights.size());
    for (const auto& armor : detected) {
        auto [x, y, z, yaw, nx, ny, nz] = pipeline.pnp.processArmorCorners(
            &pipeline.calibration, false, frame, armor, armor.armor_id);
        if (!(z > 0.0) || !std::isfinite(x+y+z+yaw)) continue;
        auto& output = result.observations[result.count++];
        pipeline.observed_poses.push_back(pipeline.pnp.pose());
        output.id = armor.armor_id;
        output.confidence = armor.res.confidence;
        output.position_camera_mm[0] = x;
        output.position_camera_mm[1] = y;
        output.position_camera_mm[2] = z;
        output.yaw_deg = yaw;
        const Eigen::Vector3d normal(nx,ny,nz);
        pipeline.observed_normals_camera.push_back(normal.allFinite() ? normal : Eigen::Vector3d(0,0,0));
        const cv::Point2f corners[] = {armor.light1_up, armor.light1_down,
                                      armor.light2_up, armor.light2_down};
        for (int i=0; i<4; ++i) {
            output.corners[2*i] = corners[i].x;
            output.corners[2*i+1] = corners[i].y;
        }
        const Eigen::Vector3d world = rotation * Eigen::Vector3d(x, y, z) / 1000.0;
        const double heading = std::atan2(rotation(1, 2), rotation(0, 2));
        const double world_yaw = pipeline.transform.armor_yaw_from_normal(normal,Eigen::Vector3d::Zero(),
            dt46_vision::normalize_angle(heading - yaw*dt46_vision::DEG2RAD));
        // Match the ROS node's color gate even when the detector displays both colors.
        if ((pipeline.tracker.target_color == 0 && armor.armor_id < 6) ||
            (pipeline.tracker.target_color == 1 && armor.armor_id >= 6)) continue;
        armors.emplace_back(armor.armor_id, world.x(), world.y(), world.z(), world_yaw);
    }
}

void save_snapshot(Pipeline& pipeline, const Eigen::Matrix3d& rotation,
                   const std::vector<std::pair<std::string, std::string>>& logs) {
    const auto s = pipeline.tracker.get_render_snapshot();
    cv::FileStorage json(".json", cv::FileStorage::WRITE | cv::FileStorage::MEMORY |
                                  cv::FileStorage::FORMAT_JSON);
    json << "spin" << int(s.spin) << "bullet_speed_m_s" << s.bullet_speed;
    json << "observation_normals_camera" << "[";
    for(const auto& n:pipeline.observed_normals_camera)json<<"["<<n.x()<<n.y()<<n.z()<<"]";
    json << "]";
    json << "pnp_armors" << "[";
    for(const auto& pose:pipeline.observed_poses){
        json<<"{"<<"reprojection_error_px"<<pose.reprojection_error_px<<"corners_camera_m"<<"[";
        for(const auto& p:pose.corners_camera_mm)json<<"["<<p.x*.001<<p.y*.001<<p.z*.001<<"]";
        json<<"]"<<"}";
    }
    json<<"]";
    json << "frame_interval_s" << pipeline.frame_interval_s << "frame_reinitialized" << int(pipeline.frame_reinitialized);
    json << "body_ready" << int(s.body_ready) << "body_ekf_state" << "[";
    for(int i=0;i<11;++i)json<<s.body_state[i];json<<"]"<<"body_covariance_diagonal"<<"[";
    for(int i=0;i<11;++i)json<<s.body_covariance(i,i);json<<"]";
    json<<"observed_plate"<<s.observed_plate<<"matched_plate_mask"<<int(s.body_matched_mask);
    json<<"aim_hit_probability"<<s.aim_hit_probability;
    json<<"aim_lateral_sigma_m"<<s.aim_lateral_sigma<<"aim_vertical_sigma_m"<<s.aim_vertical_sigma;
    json<<"fire_blockers"<<"[";for(const auto& reason:s.fire_blockers)json<<reason;json<<"]";
    if(s.tracker_state!=dt46_vision::TrackerState::LOST){
        const auto& body=s.body_state;
        const Eigen::Vector3d center(body[0],body[2],body[4]+body[10]*.5);
        const auto center_camera=(rotation.transpose()*center).eval();
        json<<"body_center_camera_m"<<"["<<center_camera[0]<<center_camera[1]<<center_camera[2]<<"]";
        json<<"body_plates"<<"[";
        for(int i=0;i<4;++i){
            const double angle=body[6]+i*dt46_vision::DEG2RAD*90,radius=body[8+i%2];
            const Eigen::Vector3d position(body[0]-radius*std::cos(angle),body[2]-radius*std::sin(angle),body[4]+(i%2)*body[10]);
            const Eigen::Vector3d tangent(-std::sin(angle),std::cos(angle),0);
            json<<"{"<<"index"<<i<<"matched"<<int((s.body_matched_mask>>i)&1)<<"selected"<<int(s.target&&s.target->index==i)<<"corners_camera_m"<<"[";
            for(const auto& signs:std::vector<Eigen::Vector2d>{{-1,-1},{1,-1},{1,1},{-1,1}}){
                const auto corner=(rotation.transpose()*(position+signs[0]*.0675*tangent+signs[1]*.0625*Eigen::Vector3d::UnitZ())).eval();
                json<<"["<<corner[0]<<corner[1]<<corner[2]<<"]";
            }
            json<<"]"<<"}";
        }
        json<<"]";
    }
    json << "yaw_tolerance_deg" << s.yaw_tolerance_deg << "pitch_tolerance_deg" << s.pitch_tolerance_deg;
    json << "gravity_m_s2" << 9.81 << "quadratic_drag_per_m" << pipeline.tracker.k_v2;
    json << "gimbal_control" << "[" << std::get<0>(s.gimbal_control)
         << std::get<1>(s.gimbal_control) << std::get<2>(s.gimbal_control) << "]";
    json << "yaw_armors" << "[";
    for (const auto& armor : s.debug_yaw_armors) {
        const Eigen::Vector3d point = rotation.transpose()*armor.pos;
        json << "{" << "position_camera_m" << "[" << point[0] << point[1] << point[2] << "]"
             << "yaw_rad" << armor.yaw << "}";
    }
    json << "]";
    json << "logs" << "[";
    for (const auto& [key,value] : logs) json << "{" << "name" << key << "value" << value << "}";
    json << "]";
    if (s.tracker_state != dt46_vision::TrackerState::LOST) {
        json << "ekf_state" << "[";
        for (int i=0; i<9; ++i) json << s.target_state[i];
        json << "]" << "ekf_yaw_velocity_rad_s" << s.ekf_yaw_vel;
        json << "ekf_second_radius_m" << s.another_r << "ekf_alternate_height_m" << s.dz;
        if (s.target) {
            const Eigen::Vector3d point = rotation.transpose()*s.target->pos;
            json << "target_camera_m" << "[" << point[0] << point[1] << point[2] << "]";
            json << "target_distance_m" << s.target->dist << "target_angle_diff_deg" << s.target->angle_diff*dt46_vision::RAD2DEG;
        }
        if (s.muzzle_target) {
            const Eigen::Vector3d point = rotation.transpose()*s.muzzle_target->pos;
            json << "muzzle_target_camera_m" << "[" << point[0] << point[1] << point[2] << "]";
        }
    }
    pipeline.snapshot = json.releaseAndGetString();
}
} // namespace

const char* dt46_error() { return last_error.c_str(); }
int dt46_result_size() { return sizeof(AimResult); }
const char* dt46_snapshot(void* context) { return static_cast<Pipeline*>(context)->snapshot.c_str(); }

int dt46_debug_image(void* context, int kind, uint8_t* pixels, int capacity,
                     int* width, int* height, int* channels) {
    try {
        auto& detector = *static_cast<Pipeline*>(context)->detector;
        if (!detector.display_mode) {
            *width = *height = *channels = 0;
            return 0;
        }
        cv::Mat frame;
        switch (kind) {
            case 0:
                if (detector.img_drawn.empty() && !detector.img.empty()) detector.draw_img();
                frame = detector.img_drawn; break;
            case 1: frame = detector.crop_img; break;
            case 2: frame = detector.img_binary; break;
            case 3: frame = detector.img_armor; break;
            case 4: frame = detector.img_armor_processed; break;
            case 5: frame = detector.img; break;
            case 6: frame = detector.gray_img_; break;
            default: throw std::invalid_argument("Unknown detector debug image");
        }
        *width=frame.cols; *height=frame.rows; *channels=frame.channels();
        const int size = frame.total()*frame.elemSize();
        if (!pixels || !size) return size;
        if (capacity < size) throw std::invalid_argument("Debug image buffer too small");
        cv::Mat destination(frame.rows, frame.cols, frame.type(), pixels);
        if (frame.channels()==3) cv::cvtColor(frame, destination, cv::COLOR_BGR2RGB);
        else frame.copyTo(destination);
        return size;
    } catch (const std::exception& error) { last_error=error.what(); return -1; }
}

void* dt46_create(const char* model, const char* parameters) {
    try {
        last_error.clear();
        return new Pipeline(model, parameters);
    } catch (const std::exception& error) {
        last_error = error.what();
        return nullptr;
    }
}

void dt46_destroy(void* context) { delete static_cast<Pipeline*>(context); }

int dt46_process(void* context, const uint8_t* rgb, int width, int height,
                 double timestamp, const double* k, const double* d,
                 const double* optical_rotation, const double* muzzle_offset,
                 int target_color, double measured_speed, AimResult* result) {
    try {
        last_error.clear();
        auto& pipeline = *static_cast<Pipeline*>(context);
        if (!(timestamp > pipeline.previous_time)) throw std::invalid_argument("Frame timestamps must increase");
        *result = AimResult{};
        if (pipeline.previous_color != target_color) {
            pipeline.tracker.tracker_state = dt46_vision::TrackerState::LOST;
            pipeline.previous_color = target_color;
            pipeline.smooth_ready = false;
        }
        pipeline.tracker.target_color = target_color;
        pipeline.detector->update_detect_color(pipeline.detect_color < 0 ? target_color : pipeline.detect_color);
        if (measured_speed > 10.0 && std::abs(measured_speed-pipeline.tracker.bullet_speed) > 0.1) {
            pipeline.tracker.bullet_speed = measured_speed;
            pipeline.tracker.build_ballistic_lut();
        }
        DT46_VISION::CameraCalibration calibration;
        std::copy(k, k+9, calibration.k.begin());
        calibration.d.assign(d, d+5);
        if (calibration.k != pipeline.calibration.k || calibration.d != pipeline.calibration.d) {
            pipeline.pnp.resetCameraInfo();
            pipeline.calibration = calibration;
            pipeline.transform.set_camera_info(cv::Mat(3, 3, CV_64F, const_cast<double*>(k)),
                cv::Mat(1, 5, CV_64F, const_cast<double*>(d)), width, height, false);
        }
        Eigen::Matrix3d rotation;
        for (int row=0; row<3; ++row)
            for (int col=0; col<3; ++col) rotation(row, col) = optical_rotation[3*row+col];
        pipeline.transform.set_optical_rotation(rotation);
        pipeline.tracker.cam_to_gun_pos = Eigen::Vector3d(muzzle_offset[0], muzzle_offset[1], muzzle_offset[2]);
        cv::Mat source(height, width, CV_8UC3, const_cast<uint8_t*>(rgb));
        cv::cvtColor(source, pipeline.bgr, cv::COLOR_RGB2BGR);
        std::vector<dt46_vision::Armor> armors;
        observe(pipeline, *result, pipeline.bgr, rotation, armors);
        const auto step=pipeline.frame_clock.advance(timestamp);
        if(!step.accepted)throw std::invalid_argument("Frame timestamps must increase");
        if(step.reset){pipeline.tracker.tracker_state=dt46_vision::TrackerState::LOST;pipeline.smooth_ready=false;}
        const double dt=step.dt;
        pipeline.frame_interval_s=dt;pipeline.frame_reinitialized=step.reset;
        pipeline.previous_time = timestamp;
        // These public caches only feed rendering; clear them so an early return cannot export an old aim.
        pipeline.tracker.target_cam_cache_.reset();
        pipeline.tracker.target_muzzle_cache_.reset();
        pipeline.tracker.gimbal_control_cache_ = {0.0, 0.0, false};
        const auto tracked = pipeline.tracker.track(pipeline.transform, armors, Eigen::Vector3d::Zero(), dt);
        save_snapshot(pipeline, rotation, std::get<1>(tracked));
        const auto& command = std::get<0>(tracked);
        const double raw_yaw = command[0], raw_pitch = command[1];
        if (const auto& target = pipeline.tracker.target_muzzle_cache_; target) {
            // Read validity through the original, stateless solver instead of changing Tracker.
            const auto [yaw, pitch, valid] = pipeline.tracker.solve_ballistic(
                pipeline.transform, *target, pipeline.tracker.cam_to_gun_rpy, Eigen::Vector3d::Zero());
            result->aim_valid = valid && std::isfinite(yaw) && std::isfinite(pitch);
        }
        result->yaw_command_deg = raw_yaw;
        result->pitch_command_deg = raw_pitch;
        result->can_fire = command[2] != 0.0;
        result->yaw_error_deg = raw_yaw;
        result->pitch_error_deg = raw_pitch;
        result->state = static_cast<int>(pipeline.tracker.tracker_state);
        result->tracked_id = result->state ? pipeline.tracker.tracked_id : -1;
        const auto position = pipeline.tracker.plate_position();
        const auto velocity = pipeline.tracker.plate_velocity();
        for (int i=0; i<3; ++i) { result->position[i]=position[i]; result->velocity[i]=velocity[i]; }
        return 0;
    } catch (const std::exception& error) {
        last_error = error.what();
        return -1;
    }
}
