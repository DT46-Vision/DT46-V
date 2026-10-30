#include "modules/vision/pipeline.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>

namespace dt46 {
namespace {
const std::set<std::string> detector_parameters = {
    "light_area_min", "light_h_w_ratio", "light_angle_max", "light_color_difference",
    "height_rate_tol", "height_multiplier_min", "height_multiplier_max", "crop_height", "crop_margin", "presence_threshold",
    "roi_crop", "roi_scale", "binary_val", "display_mode", "detect_color", "print_period_ms"};
const std::set<std::string> tracker_parameters = {
    "max_match_distance", "max_match_yaw_diff", "jump_cooldown_max", "tracking_thres", "lost_thres", "dist_tol",
    "ekf_QR_q_xyz", "ekf_QR_q_yaw", "ekf_QR_q_r", "ekf_QR_r_xyz_factor", "ekf_QR_r_depth_factor", "ekf_QR_r_yaw", "ekf_QR_stable_dist",
    "radius_r_min", "radius_r_max", "system_delay", "controller_delay", "bullet_speed", "k_v2", "gravity", "shootable_dist",
    "yaw_tolerance_deg", "pitch_tolerance_deg", "min_hit_probability", "min_spinning_vel", "spin_enter_time", "spin_exit_time",
    "cam_to_gun_rpy_r", "cam_to_gun_rpy_p", "cam_to_gun_rpy_y", "target_color", "follow_decision",
    "display", "display_fps_limit", "text_size"};
const std::vector<std::string> debug_images = {"result", "crop_img", "bin_img", "model_input", "img_debug", "gray"};
Json node_json(const cv::FileNode& node) {
    if (node.isMap()) {
        Json result = Json::object();
        for (const auto& item : node) result[item.name()] = node_json(item);
        return result;
    }
    if (node.isSeq()) {
        Json result = Json::array();
        for (const auto& item : node) result.push_back(node_json(item));
        return result;
    }
    if (node.isInt()) return int(node);
    if (node.isReal()) return double(node);
    if (node.isString()) {
        const auto value = std::string(node);
        if (value == "true") return true;
        if (value == "false") return false;
        return value;
    }
    return nullptr;
}
std::runtime_error native_error() { return std::runtime_error(dt46_error()); }
void validate_settings(const Json& settings) {
    const auto& d = settings.at("detector"); const auto& t = settings.at("tracker");
    const auto number = [](const Json& value) { return value.is_boolean() ? double(value.get<bool>()) : value.get<double>(); };
    for (const auto* group : {&d, &t}) for (const auto& value : *group)
        if ((!value.is_boolean() && !value.is_number()) || !std::isfinite(number(value))) throw std::invalid_argument("Parameters must be finite numbers");
    if (!(number(d.at("roi_scale")) > 0 && number(d.at("roi_scale")) <= 1) ||
        number(d.at("binary_val")) < 0 || number(d.at("binary_val")) > 255) throw std::invalid_argument("Invalid ROI scale/threshold");
    for (const char* key : {"crop_height", "crop_margin", "light_h_w_ratio", "height_rate_tol"})
        if (!(number(d.at(key)) > 0)) throw std::invalid_argument(std::string(key)+" must be positive");
    if(number(d.at("presence_threshold"))<=0||number(d.at("presence_threshold"))>1||
       number(d.at("crop_margin"))<1||number(d.at("light_angle_max"))<0||number(d.at("light_angle_max"))>90||
       number(d.at("light_color_difference"))<0||number(d.at("light_color_difference"))>255||
       number(d.at("light_area_min"))<0||number(d.at("height_multiplier_min"))<0||
       number(d.at("height_multiplier_min"))>=number(d.at("height_multiplier_max")))
        throw std::invalid_argument("Invalid candidate/model bounds");
    if (!(number(t.at("bullet_speed")) > 0 && number(t.at("gravity")) > 0) || number(t.at("k_v2")) < 0 ||
        !(number(t.at("radius_r_min")) > 0) || number(t.at("radius_r_min")) > number(t.at("radius_r_max")))
        throw std::invalid_argument("Invalid tracker physics/radius");
    if(t.contains("min_hit_probability") && (number(t.at("min_hit_probability"))<=0 || number(t.at("min_hit_probability"))>1))
        throw std::invalid_argument("min_hit_probability must be in (0,1]");
    for (const char* key : {"tracking_thres", "lost_thres", "jump_cooldown_max"}) {
        const double value = number(t.at(key));
        if (value < 0 || value != std::floor(value)) throw std::invalid_argument(std::string(key)+" must be a nonnegative integer");
    }
    for (const char* key : {"system_delay", "controller_delay", "spin_enter_time", "spin_exit_time"})
        if (number(t.at(key)) < 0) throw std::invalid_argument(std::string(key)+" must be nonnegative");
    for (const char* key : {"ekf_QR_q_xyz", "ekf_QR_q_yaw", "ekf_QR_q_r", "ekf_QR_r_xyz_factor", "ekf_QR_r_depth_factor"})
        if(t.contains(key)&&number(t.at(key))<0)throw std::invalid_argument(std::string(key)+" must be nonnegative");
    for (const char* key : {"ekf_QR_r_yaw", "ekf_QR_stable_dist", "max_match_distance", "max_match_yaw_diff"})
        if(number(t.at(key))<=0)throw std::invalid_argument(std::string(key)+" must be positive");
    for (const auto* group : {&d, &t}) for (const char* key : {"roi_crop", "display_mode", "follow_decision", "display", "display_fps_limit"})
        if (group->contains(key) && number(group->at(key)) != 0 && number(group->at(key)) != 1)
            throw std::invalid_argument(std::string(key)+" must be boolean");
    const double color=number(d.at("detect_color")), target=number(t.at("target_color")), period=number(d.at("print_period_ms"));
    if (color < -1 || color > 2 || color != std::floor(color) || (target != 0 && target != 1))
        throw std::invalid_argument("Invalid detector/tracker color");
    if (period < 1 || period != std::floor(period) || number(t.at("text_size")) <= 0 || number(t.at("text_size")) > 10)
        throw std::invalid_argument("Invalid log period/text size");
}
template<int Rows, int Cols> Json matrix_json(const cv::Matx<double, Rows, Cols>& matrix) {
    Json result = Json::array();
    for (int i = 0; i < Rows; ++i) {
        Json row = Json::array();
        for (int j = 0; j < Cols; ++j) row.push_back(matrix(i,j));
        result.push_back(row);
    }
    return result;
}
}

Json read_yaml(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open " + path.string());
    std::string source((std::istreambuf_iterator<char>(input)), {});
    if (!source.starts_with("%YAML")) source = "%YAML:1.0\n---\n" + source;
    cv::FileStorage file(source, cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_YAML);
    if (!file.isOpened()) throw std::runtime_error("Cannot parse " + path.string());
    return node_json(file.root());
}

AimPipeline::AimPipeline(std::filesystem::path root, const std::string& profile, bool debug) : root_(std::move(root)) {
    if (profile != "infantry" && profile != "sentry" && profile != "hero") throw std::invalid_argument("Unknown vision profile");
    std::ifstream model_file(root_/"standalone/armor_model.json");
    if(!model_file)throw std::runtime_error("Missing native armor model configuration");
    const auto model_defaults=Json::parse(model_file);
    const auto& detector=model_defaults.at("detector");
    auto tracker = read_yaml(root_/"src/rm_tracker/config"/("tracker_params_"+profile+".yaml")).at("rm_tracker").at("ros__parameters");
    Json defaults = {{"controller_delay", .02}, {"k_v2", .019}, {"gravity", 9.81}, {"spin_enter_time", .3}, {"spin_exit_time", .15}};
    defaults.update(tracker); tracker = std::move(defaults);
    settings_ = {{"detector", Json::object()}, {"tracker", Json::object()}, {"profile", profile}, {"model", model_defaults.at("model")}};
    for (const auto& name : detector_parameters) if (detector.contains(name)) settings_["detector"][name] = detector.at(name);
    for (const auto& name : tracker_parameters) if (tracker.contains(name)) settings_["tracker"][name] = tracker.at(name);
    settings_["detector"]["display_mode"] = debug;
    // -1 retains the native adapter's existing decision-color filtering.
    settings_["detector"]["detect_color"] = -1;
    settings_["detector"]["print_period_ms"] = detector.value("print_period_ms",1000);
    settings_["tracker"]["display"] = debug;
    settings_["tracker"]["display_fps_limit"] = tracker.value("display_fps_limit",true);
    settings_["tracker"]["text_size"] = tracker.value("text_size",1.0);
    settings_["tracker"]["follow_decision"] = true;
    settings_["tracker"]["target_color"] = tracker.value("target_color",0);
    reset();
}

AimPipeline::~AimPipeline() { if (context_) dt46_destroy(context_); }

void* AimPipeline::create(const Json& settings) const {
    validate_settings(settings);
    Json parameters = {{"detector", settings.at("detector")}, {"tracker", settings.at("tracker")}};
    for (auto& group : parameters) for (auto& value : group) if (value.is_boolean()) value = int(value.get<bool>());
    const auto model = root_/settings.at("model").get<std::string>();
    auto* created = dt46_create(model.string().c_str(), parameters.dump().c_str());
    if (!created) throw native_error();
    return created;
}
void AimPipeline::reset() {
    auto* replacement = create(settings_);
    if (context_) dt46_destroy(context_);
    context_ = replacement;
    ++revision_;
}
void AimPipeline::configure(const Json& changes) {
    if (!changes.is_object()) throw std::invalid_argument("Configuration changes must be an object");
    auto next = settings_;
    for (const auto& [group, values] : changes.items()) {
        if (group == "model") { next[group] = values.get<std::string>(); continue; }
        if ((group != "detector" && group != "tracker") || !values.is_object()) throw std::invalid_argument("Invalid parameter group");
        const auto& allowed = group == "detector" ? detector_parameters : tracker_parameters;
        for (const auto& [key, value] : values.items()) {
            if (!allowed.contains(key)) throw std::invalid_argument("Unknown parameter: " + key);
            next[group][key] = value;
        }
    }
    auto* replacement = create(next);
    if (context_) dt46_destroy(context_);
    context_ = replacement;
    settings_ = std::move(next);
    ++revision_;
}

Json AimPipeline::process(const cv::Mat& rgb, double timestamp, const cv::Matx33d& k,
                          const cv::Vec<double,5>& distortion, const cv::Matx33d& optical_rotation,
                          const cv::Vec3d& muzzle_offset, int target_color, double measured_speed) {
    if (rgb.type() != CV_8UC3 || rgb.empty()) throw std::invalid_argument("Expected a nonempty RGB8 image");
    if ((target_color != 0 && target_color != 1) || !std::isfinite(timestamp) || timestamp < 0 ||
        !std::isfinite(measured_speed) || measured_speed < 0 || !(k(0,0) > 0 && k(1,1) > 0))
        throw std::invalid_argument("Invalid frame metadata");
    for (double value : k.val) if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite intrinsics");
    if(k(0,1)!=0||k(1,0)!=0||k(2,0)!=0||k(2,1)!=0||k(2,2)!=1)
        throw std::invalid_argument("Armor model requires zero-skew camera intrinsics");
    for (double value : distortion.val) if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite distortion");
    for (double value : optical_rotation.val) if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite optical rotation");
    for (double value : muzzle_offset.val) if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite muzzle offset");
    const cv::Mat contiguous = rgb.isContinuous() ? rgb : rgb.clone();
    const auto& tracker=settings_.at("tracker");
    const auto number=[](const Json& v){return v.is_boolean()?double(v.get<bool>()):v.get<double>();};
    if (!number(tracker.at("follow_decision"))) target_color=int(number(tracker.at("target_color")));
    const int configured_color=int(number(settings_.at("detector").at("detect_color")));
    AimResult result{};
    if (dt46_process(context_, contiguous.data, rgb.cols, rgb.rows, timestamp, k.val, distortion.val,
                     optical_rotation.val, muzzle_offset.val, target_color, measured_speed, &result)) throw native_error();
    const char* states[] = {"LOST", "DETECTING", "TRACKING", "TEMP_LOST"};
    const Json debug=Json::parse(dt46_snapshot(context_));
    Json observations = Json::array();
    for (int i = 0; i < result.count; ++i) {
        const auto& a = result.observations[i];
        Json corners = Json::array();
        for (int j = 0; j < 4; ++j) corners.push_back({a.corners[2*j], a.corners[2*j+1]});
        observations.push_back({{"armor_id", a.id}, {"presence_score", a.confidence},
            {"position_camera_m", {a.position_camera_mm[0]/1000, a.position_camera_mm[1]/1000, a.position_camera_mm[2]/1000}},
            {"yaw_deg", a.yaw_deg}, {"normal_camera",debug.at("observation_normals_camera").at(i)}, {"corners_px", corners},
            {"model",debug.at("model_armors").at(i)}});
    }
    return {{"simulation_time_s", timestamp}, {"state", states[result.state]}, {"tracked_id", result.tracked_id},
        {"can_fire", bool(result.can_fire)}, {"yaw_error_deg", result.yaw_error_deg}, {"pitch_error_deg", result.pitch_error_deg},
        {"aim", {{"valid", bool(result.aim_valid)}, {"yaw_error_deg", result.yaw_command_deg},
                 {"pitch_error_deg", result.pitch_command_deg}, {"can_fire", bool(result.can_fire)}}},
        {"position_camera_origin_world_axes_m", result.position}, {"velocity_world_axes_m_s", result.velocity},
        {"light_count", result.light_count}, {"candidate_count",debug.at("candidate_count")}, {"observations", observations}, {"tracker_debug", debug},
        {"input", {{"K", matrix_json(k)}, {"D", distortion.val}, {"optical_rotation", matrix_json(optical_rotation)},
                    {"muzzle_offset_camera_m", muzzle_offset.val}, {"target_color",target_color},
                    {"detect_color",configured_color < 0 ? target_color : configured_color}}}};
}
cv::Mat AimPipeline::debug_image(const std::string& name) const {
    const auto found = std::find(debug_images.begin(), debug_images.end(), name);
    if (found == debug_images.end()) throw std::invalid_argument("Unknown debug image");
    const int kind = int(found-debug_images.begin());
    int width=0, height=0, channels=0;
    const int size = dt46_debug_image(context_, kind, nullptr, 0, &width, &height, &channels);
    if (size < 0) throw native_error();
    if (!size) return {};
    cv::Mat result(height, width, CV_MAKETYPE(CV_8U, channels));
    if (dt46_debug_image(context_, kind, result.data, size, &width, &height, &channels) < 0) throw native_error();
    return result;
}
}
