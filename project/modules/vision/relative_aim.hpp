#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <deque>
#include <optional>
#include <vector>

namespace dt46::relative {
using Vec3 = Eigen::Vector3d;
struct ImuSample {
    double published_s;
    Eigen::Quaterniond orientation;
};
struct PlateObservation {
    Vec3 camera_position_m;
    Vec3 camera_normal;
};
struct Frame {
    double image_host_stamp_s, received_s, detection_done_s;
    ImuSample visual_imu;
    std::vector<PlateObservation> plates;
};
struct Body {
    double stamp_s = 0, phase = 0, omega = 0, residual_m = 0;
    Vec3 center = Vec3::Zero(), velocity = Vec3::Zero();
    std::array<double, 2> radius{}, height{};
};
struct DelayConfig {
    double image_stamp_delay_s = 0;
    double imu_publish_delay_s = 0;
};
// Geometry phase and mounting are supplied assumptions, never simulation metadata.
class Tracker {
public:
    Tracker(double adjacent_phase_rad, Eigen::Matrix3d camera_to_imu, DelayConfig delays = {});
    bool update(const Frame& frame);
    const std::optional<Body>& body() const { return body_; }
    Vec3 plate(int index, double ahead_s) const;
    const char* reason() const { return reason_; }
private:
    struct Sample { double t, phase, yaw; int group; Vec3 position; };
    double adjacent_, last_stamp_ = -1, phase_ = 0, omega_ = 0;
    Eigen::Matrix3d mounting_;
    DelayConfig delays_;
    std::optional<ImuSample> previous_imu_;
    std::deque<Sample> samples_;
    std::optional<Body> body_;
    const char* reason_ = "uninitialized";
    bool fit(double stamp);
};
struct Timing {
    double solve_s;
    // Unmeasured bounds stay absent. They are not execution acknowledgements.
    std::optional<double> stamp_error_bound_s, execution_delay_s;
};
struct Shot {
    bool valid = false, can_fire = false;
    int plate = -1;
    double yaw_rad = 0, pitch_rad = 0, flight_s = 0, model_miss_m = 0;
    const char* reason = "no_model";
    Vec3 intercept = Vec3::Zero();
};
// Vacuum, ideal gimbal experiment only. No drag or muzzle dynamics are implied.
Shot aim(const Tracker& tracker, const Timing& timing, double speed_m_s,
         double speed_received_s, const Vec3& bore_direction);
}
