#include "aim.h"
#include "armor_model.hpp"
#include "rm_tracker/tracker.hpp"
#include "rm_tracker/frame_clock.hpp"
#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>

namespace {
thread_local std::string last_error;
using Clock = std::chrono::steady_clock;
double elapsed_ms(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

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
        throw std::invalid_argument("Ballistic model requires gravity = 9.81 m/s^2");
    tracker.cam_to_gun_rpy << parameter(config, "cam_to_gun_rpy_r", 0.0),
                              parameter(config, "cam_to_gun_rpy_p", 0.0),
                              parameter(config, "cam_to_gun_rpy_y", 0.0);
}

struct Pipeline {
    std::unique_ptr<dt46::ArmorModel> detector;
    cv::Matx33d calibration_k{};
    cv::Vec<double,5> calibration_d{};
    dt46_vision::Tracker tracker;
    dt46_vision::RmTF transform;
    double previous_time = -1.0;
    dt46_vision::FrameClock frame_clock;
    double frame_interval_s = 0;
    bool frame_reinitialized = true;
    int previous_color = -1;
    int detect_color = -1;
    cv::Mat rgb_frame;
    std::string snapshot = "{}";
    double stage_ms[5]{};
    std::vector<Eigen::Vector3d> observed_normals_camera;
    std::vector<dt46::ModelObservation> observed_poses;

    Pipeline(const char* model, const char* parameters) {
        cv::setNumThreads(1);
        cv::FileStorage config(parameters, cv::FileStorage::READ | cv::FileStorage::MEMORY |
                                           cv::FileStorage::FORMAT_JSON);
        const auto source = config["detector"];
        detector = std::make_unique<dt46::ArmorModel>(model,source);
        detect_color = int(parameter(source,"detect_color",-1));
        configure_tracker(tracker, config["tracker"]);
    }
};

void observe(Pipeline& pipeline, AimResult& result, const cv::Mat& frame,
             const Eigen::Matrix3d& rotation, std::vector<dt46_vision::Armor>& armors) {
    const int color=pipeline.detect_color<0?pipeline.tracker.target_color:pipeline.detect_color;
    pipeline.observed_poses=pipeline.detector->detect(frame,pipeline.calibration_k,pipeline.calibration_d,color);
    pipeline.observed_normals_camera.clear();
    result.light_count=pipeline.detector->light_count;
    for(const auto& armor:pipeline.observed_poses){
        auto& output=result.observations[result.count++];
        output.id=armor.armor_id;output.confidence=armor.presence;
        for(int i=0;i<3;++i)output.position_camera_mm[i]=armor.position[i]*1000;
        const Eigen::Vector3d normal(armor.rotation(0,2),armor.rotation(1,2),armor.rotation(2,2));
        pipeline.observed_normals_camera.push_back(normal);
        output.yaw_deg=std::atan2(-normal.x(),-normal.z())*dt46_vision::RAD2DEG;
        for(int i=0;i<4;++i){output.corners[2*i]=armor.corners[i].x;output.corners[2*i+1]=armor.corners[i].y;}
        if(!armor.vehicle||armor.team!=pipeline.tracker.target_color)continue;
        const Eigen::Vector3d world=rotation*Eigen::Vector3d(armor.position[0],armor.position[1],armor.position[2]);
        const double world_yaw=pipeline.transform.armor_yaw_from_normal(normal,Eigen::Vector3d::Zero(),0);
        armors.emplace_back(armor.armor_id,world.x(),world.y(),world.z(),world_yaw);
    }
    pipeline.stage_ms[1]=pipeline.detector->candidate_ms;
    pipeline.stage_ms[2]=pipeline.detector->model_ms;
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
    json << "model_armors" << "[";
    for(const auto& pose:pipeline.observed_poses){
        const double range=cv::norm(pose.position);
        const cv::Vec3d normal(pose.rotation(0,2),pose.rotation(1,2),pose.rotation(2,2));
        json<<"{"<<"sticker_type"<<pose.sticker<<"team"<<(pose.team==0?"red":"blue")
            <<"robot_number"<<pose.robot_number<<"target_family"<<(pose.vehicle?"vehicle":"structure")
            <<"presence_score"<<pose.presence<<"sticker_score"<<pose.sticker_score<<"team_score"<<pose.team_score
            <<"tracker_eligible"<<int(pose.vehicle&&pose.team==pipeline.tracker.target_color)
            <<"depth_m"<<pose.position[2]<<"range_m"<<range
            <<"bearing_yaw_deg"<<std::atan2(pose.position[0],pose.position[2])*dt46_vision::RAD2DEG
            <<"bearing_pitch_deg"<<std::atan2(-pose.position[1],std::hypot(pose.position[0],pose.position[2]))*dt46_vision::RAD2DEG
            <<"incidence_deg"<<std::acos(std::clamp(normal.dot(-pose.position/range),-1.,1.))*dt46_vision::RAD2DEG
            <<"roi_xywh"<<"["<<pose.roi.x<<pose.roi.y<<pose.roi.width<<pose.roi.height<<"]"
            <<"rotation_camera_from_board"<<"[";
        for(int i=0;i<3;++i){json<<"[";for(int j=0;j<3;++j)json<<pose.rotation(i,j);json<<"]";}
        json<<"]"<<"}";
    }
    json<<"]"<<"candidate_count"<<pipeline.detector->candidate_count<<"invalid_pose_count"<<pipeline.detector->invalid_pose_count;
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
        if (!detector.display) {
            *width = *height = *channels = 0;
            return 0;
        }
        if(kind<0||kind>=int(detector.images.size()))throw std::invalid_argument("Unknown model debug image");
        const cv::Mat& frame=detector.images[kind];
        *width=frame.cols; *height=frame.rows; *channels=frame.channels();
        const int size = frame.total()*frame.elemSize();
        if (!pixels || !size) return size;
        if (capacity < size) throw std::invalid_argument("Debug image buffer too small");
        cv::Mat destination(frame.rows, frame.cols, frame.type(), pixels);
        frame.copyTo(destination);
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
        const auto begin_frame = Clock::now();
        last_error.clear();
        auto& pipeline = *static_cast<Pipeline*>(context);
        if (!(timestamp > pipeline.previous_time)) throw std::invalid_argument("Frame timestamps must increase");
        *result = AimResult{};
        if (pipeline.previous_color != target_color) {
            pipeline.tracker.tracker_state = dt46_vision::TrackerState::LOST;
            pipeline.previous_color = target_color;
        }
        pipeline.tracker.target_color = target_color;
        if (measured_speed > 10.0 && std::abs(measured_speed-pipeline.tracker.bullet_speed) > 0.1) {
            pipeline.tracker.bullet_speed = measured_speed;
        }
        std::copy(k,k+9,pipeline.calibration_k.val);
        std::copy(d,d+5,pipeline.calibration_d.val);
        pipeline.transform.set_camera_info(cv::Mat(pipeline.calibration_k),cv::Mat(pipeline.calibration_d),width,height,false);
        Eigen::Matrix3d rotation;
        for (int row=0; row<3; ++row)
            for (int col=0; col<3; ++col) rotation(row, col) = optical_rotation[3*row+col];
        pipeline.transform.set_optical_rotation(rotation);
        pipeline.tracker.cam_to_gun_pos = Eigen::Vector3d(muzzle_offset[0], muzzle_offset[1], muzzle_offset[2]);
        cv::Mat source(height, width, CV_8UC3, const_cast<uint8_t*>(rgb));
        // Debug images outlive the caller's input buffer. Keep one owned RGB copy.
        source.copyTo(pipeline.rgb_frame);
        pipeline.stage_ms[0] = elapsed_ms(begin_frame);
        std::vector<dt46_vision::Armor> armors;
        observe(pipeline, *result, pipeline.rgb_frame, rotation, armors);
        const auto step=pipeline.frame_clock.advance(timestamp);
        if(!step.accepted)throw std::invalid_argument("Frame timestamps must increase");
        if(step.reset)pipeline.tracker.tracker_state=dt46_vision::TrackerState::LOST;
        const double dt=step.dt;
        pipeline.frame_interval_s=dt;pipeline.frame_reinitialized=step.reset;
        pipeline.previous_time = timestamp;
        const auto begin_track = Clock::now();
        const auto tracked = pipeline.tracker.track(pipeline.transform, armors, Eigen::Vector3d::Zero(), dt);
        pipeline.stage_ms[3] = elapsed_ms(begin_track);
        const auto begin_snapshot = Clock::now();
        save_snapshot(pipeline, rotation, std::get<1>(tracked));
        pipeline.stage_ms[4] = elapsed_ms(begin_snapshot);
        const auto& command = std::get<0>(tracked);
        const double raw_yaw = command[0], raw_pitch = command[1];
        result->aim_valid = pipeline.tracker.aim_valid();
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

void dt46_stage_times(const void* context, double* milliseconds) {
    const auto& pipeline = *static_cast<const Pipeline*>(context);
    std::copy(std::begin(pipeline.stage_ms), std::end(pipeline.stage_ms), milliseconds);
}
