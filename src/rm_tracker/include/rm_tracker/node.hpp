#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cv_bridge/cv_bridge.h>

// 替换为你实际的接口包名，比如 rm_interfaces 或者 dt_interfaces
#include <rm_interfaces/msg/armors_msg.hpp>
#include <rm_interfaces/msg/decision.hpp>
#include <rm_interfaces/msg/gimbal_control.hpp>

#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <queue>
#include <chrono>

#include "rm_tracker/tracker.hpp"
#include "rm_tracker/rm_tf_tools.hpp"

namespace dt46_vision {

// 简单的日志节流器结构
struct LogThrottler {
    rclcpp::Logger logger;
    std::chrono::milliseconds throttle_ms;
    std::chrono::steady_clock::time_point last_log_time;

    LogThrottler(rclcpp::Logger l, int ms)
        : logger(l), throttle_ms(ms), last_log_time(std::chrono::steady_clock::now()) {}

    bool should_log() {
        auto now = std::chrono::steady_clock::now();
        if (now - last_log_time >= throttle_ms) {
            last_log_time = now;
            return true;
        }
        return false;
    }
};

// 用于队列传递的内部数据结构
struct TrackData {
    rm_interfaces::msg::ArmorsMsg::SharedPtr msg;
    Eigen::Vector3d imu_rpy;
};

// 渲染快照结构体
struct RenderSnapshot {
    TrackerState tracker_state;
    Eigen::Matrix<double, 9, 1> target_state;
    double another_r;
    double dz;
    std::vector<Armor> debug_yaw_armors;
    std::optional<Armor> target;
    std::optional<Armor> muzzle_target;
    bool spin;
    double yaw_tolerance_deg;
    double pitch_tolerance_deg;
    std::tuple<double, double, bool> gimbal_control;
    double ekf_yaw_vel;
    double bullet_speed;
};

class RmTrackerNode : public rclcpp::Node {
public:
    RmTrackerNode(const rclcpp::NodeOptions& options);
    ~RmTrackerNode();

private:
    // =============== 回调函数 ===============
    void imu_rpy_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void armors_cb(const rm_interfaces::msg::ArmorsMsg::SharedPtr msg);
    void camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void res_img_cb(const sensor_msgs::msg::Image::SharedPtr msg);
    void cb_opponent_color(const rm_interfaces::msg::Decision::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult param_cb(const std::vector<rclcpp::Parameter>& params);

    // =============== 核心线程函数 ===============
    void processing_worker();

    // =============== 渲染工具函数 ===============
    void draw_tracking_state(cv::Mat& draw, const RenderSnapshot& snapshot);
    void draw_aiming_hud(cv::Mat& draw, const RenderSnapshot& snapshot);
    RenderSnapshot get_render_snapshot();

    // =============== 组件与状态 ===============
    Tracker tracker_;
    RmTF tf_;

    Eigen::Vector3d imu_rpy_;
    Eigen::Vector3d rotation_rpy_;
    bool imu_initialized_ = false;

    // 参数
    int target_color_;
    bool follow_decision_;
    bool show_rpy_;
    bool debug_;
    bool display_fps_limit_;
    bool display_;
    double text_size_;

    // FPS 计算
    int process_counter_ = 0;
    std::chrono::steady_clock::time_point last_fps_log_time_;

    // =============== 多线程通信组件 ===============
    std::mutex tracker_lock_;
    std::mutex render_lock_;
    std::mutex imu_lock_;

    std::queue<TrackData> track_queue_;
    std::condition_variable cv_;
    bool worker_running_ = true;
    std::thread worker_thread_;

    std::optional<RenderSnapshot> render_snapshot_;
    std::chrono::steady_clock::time_point last_render_time_;

    // =============== ROS 订阅与发布 ===============
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_imu_rpy_;
    rclcpp::Subscription<rm_interfaces::msg::ArmorsMsg>::SharedPtr sub_armors_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_;
    rclcpp::Subscription<rm_interfaces::msg::Decision>::SharedPtr sub_opponent_color_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_img_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_tracking_state_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ballistic_img_;
    rclcpp::Publisher<rm_interfaces::msg::GimbalControl>::SharedPtr pub_gimbal_control_;

    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    std::unique_ptr<LogThrottler> logger_throttler_;
};

} // namespace dt46_vision
