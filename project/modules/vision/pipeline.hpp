#pragma once

#include "aim.h"
#include <filesystem>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <string>

namespace dt46 {
using Json = nlohmann::json;
Json read_yaml(const std::filesystem::path& path);

class AimPipeline {
public:
    AimPipeline(std::filesystem::path root, const std::string& profile = "infantry", bool debug = false);
    ~AimPipeline();
    AimPipeline(const AimPipeline&) = delete;
    AimPipeline& operator=(const AimPipeline&) = delete;
    void reset();
    void configure(const Json& changes);
    Json process(const cv::Mat& rgb, double timestamp, const cv::Matx33d& k,
                 const cv::Vec<double, 5>& distortion, const cv::Matx33d& optical_rotation,
                 const cv::Vec3d& muzzle_offset, int target_color, double measured_speed = 0);
    cv::Mat debug_image(const std::string& name) const;
    const Json& settings() const { return settings_; }
    const std::filesystem::path& root() const { return root_; }
    double nominal_speed() const { return settings_.at("tracker").at("bullet_speed"); }
    const void* context() const { return context_; }
    std::uint64_t revision() const { return revision_; }
private:
    void* create(const Json& settings) const;
    std::filesystem::path root_;
    Json settings_;
    void* context_ = nullptr;
    std::uint64_t revision_ = 0;
};
}
