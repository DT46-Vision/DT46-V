#include "rm_tracker/node.hpp"

using namespace std::chrono_literals;

namespace dt46_vision {

// =====================================================================
// 构造函数与析构函数
// =====================================================================
RmTrackerNode::RmTrackerNode(const rclcpp::NodeOptions& options)
    : Node("rm_tracker", options) {

    // 1. 从 ROS 2 参数服务器读取配置
    target_color_ = this->declare_parameter("target_color", 0);
    tracker_.bullet_speed = this->declare_parameter("bullet_speed", 28.0);
    display_ = this->declare_parameter("display", false);
    follow_decision_ = this->declare_parameter("follow_decision", false);
    display_fps_limit_ = this->declare_parameter("display_fps_limit", true);

    // 2. 加载机械外参：轴固定偏移修正（例如 pitch/yaw 轴不共轴或相机倒置补偿）
    rotation_rpy_ << this->declare_parameter("rotation_rpy_r", 0.0),
                     this->declare_parameter("rotation_rpy_p", 0.0),
                     this->declare_parameter("rotation_rpy_y", -180.0);

    // 3. 初始化日志节流器和计时器
    logger_throttler_ = std::make_unique<LogThrottler>(this->get_logger(), 1000);
    last_fps_log_time_ = std::chrono::steady_clock::now();
    last_render_time_ = std::chrono::steady_clock::now();

    // 4. 配置通信服务质量 (QoS)：使用最适合车载传感器和高频图像的 SensorData 策略（低延迟、允许丢包）
    auto sensor_qos = rclcpp::SensorDataQoS();

    // 5. 绑定并订阅高频基础数据
    sub_imu_rpy_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/imu/rpy", sensor_qos, std::bind(&RmTrackerNode::imu_rpy_cb, this, std::placeholders::_1));

    sub_armors_ = this->create_subscription<rm_interfaces::msg::ArmorsMsg>(
        "/detector/armors_info", sensor_qos, std::bind(&RmTrackerNode::armors_cb, this, std::placeholders::_1));

    sub_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", sensor_qos, std::bind(&RmTrackerNode::camera_info_cb, this, std::placeholders::_1));

    sub_opponent_color_ = this->create_subscription<rm_interfaces::msg::Decision>(
        "/nav/decision", sensor_qos, std::bind(&RmTrackerNode::cb_opponent_color, this, std::placeholders::_1));

    sub_raw_img_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", sensor_qos, std::bind(&RmTrackerNode::res_img_cb, this, std::placeholders::_1));

    // 6. 声明输出发布器
    pub_tracking_state_img_ = this->create_publisher<sensor_msgs::msg::Image>("/tracker/tracking_state_img", sensor_qos);
    pub_ballistic_img_ = this->create_publisher<sensor_msgs::msg::Image>("/tracker/ballistic_img", sensor_qos);
    pub_gimbal_control_ = this->create_publisher<rm_interfaces::msg::GimbalControl>("/tracker/gimbal_control", sensor_qos);

    // 7. 注册动态参数热重载回调
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&RmTrackerNode::param_cb, this, std::placeholders::_1));

    // 8. 强力开启物理级计算隔离：启动异步独立解算工作线程
    worker_running_ = true;
    worker_thread_ = std::thread(&RmTrackerNode::processing_worker, this);

    RCLCPP_INFO(this->get_logger(), "DT46 Tracker [C++ 纯血版] 节点启动成功。");
}

RmTrackerNode::~RmTrackerNode() {
    worker_running_ = false;
    cv_.notify_all(); // 唤醒正在阻塞等待新装甲板信号的条件变量
    if (worker_thread_.joinable()) {
        worker_thread_.join(); // 回收线程，安全退出
    }
}

// =====================================================================
// ROS 2 数据订阅回调函数
// =====================================================================

void RmTrackerNode::imu_rpy_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    Eigen::Vector3d raw_rpy(msg->vector.x, msg->vector.y, msg->vector.z);

    // 保护陀螺仪高频姿态写入时的线程安全
    std::lock_guard<std::mutex> lock(imu_lock_);
    imu_rpy_ = tf_.rotate_pose_axis(raw_rpy, rotation_rpy_);
    imu_initialized_ = true;
}

void RmTrackerNode::camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (tf_.has_camera_info()) return; // 内参通常不发生动态改变，单次读取即可

    // 将 ROS 1D 扁平数组数据强转映射为 OpenCV 兼容的双精度矩阵结构
    cv::Mat k(3, 3, CV_64F, (void*)msg->k.data());
    cv::Mat d(1, 5, CV_64F, (void*)msg->d.data());

    tf_.set_camera_info(k, d, msg->width, msg->height);
    RCLCPP_INFO(this->get_logger(), "相机内参成功加载并矫正: %dx%d", msg->width, msg->height);
}

void RmTrackerNode::armors_cb(const rm_interfaces::msg::ArmorsMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> imu_l(imu_lock_);
    if (!imu_initialized_) return; // 必须拿到第一帧陀螺仪数据之后才允许进行后续的时空同步

    TrackData data;
    data.msg = msg;
    data.imu_rpy = imu_rpy_;

    {
        std::lock_guard<std::mutex> lock(tracker_lock_);
        // 核心机制：锁定并清空阻塞旧数据队列。长度维持为1，触发跳帧，确保永远只追踪战场最新画面
        std::queue<TrackData> empty;
        std::swap(track_queue_, empty);
        track_queue_.push(data);
    }
    cv_.notify_one(); // 通知底层异步 worker 开始解析算力运算
}

// =====================================================================
// 核心解算工作线程 (Worker)
// =====================================================================
void RmTrackerNode::processing_worker() {
    while (rclcpp::ok() && worker_running_) {
        TrackData data;
        {
            std::unique_lock<std::mutex> lock(tracker_lock_);
            // 阻塞直至受到条件变量唤醒
            cv_.wait(lock, [this]() { return !track_queue_.empty() || !worker_running_; });
            if (!worker_running_) break;

            data = track_queue_.front();
            track_queue_.pop();
        }

        // 1. 将 ROS 自定义检测消息反序列化解析为我们的标准 Armor 结构体
        std::vector<Armor> raw_armors;
        for (const auto& a : data.msg->armors) {
            // mm 统一转为物理世界标准的米(m)
            Armor armor(a.armor_id, a.dx / 1000.0, a.dy / 1000.0, a.dz / 1000.0, -a.yaw);
            raw_armors.push_back(armor);
        }

        rclcpp::Time ros_clock = data.msg->header.stamp;
        double dt = 0.01; // 临时设定为固定控制周期 (根据硬件可扩展为基于 header 差分计算的真实 dt)

        // 2. 线程同步锁：开始调用核心预测状态机与滤波器
        std::tuple<double, double, bool> gimbal_cmd;
        {
            std::lock_guard<std::mutex> lock(tracker_lock_);
            process_counter_++;

            // 解包复合返回值并安全隔离底层
            auto track_result = tracker_.track(tf_, raw_armors, data.imu_rpy, dt);
            auto cmd_vec = std::get<0>(track_result);

            // 严谨校验解算向量完整度，防爆越界
            if (cmd_vec.size() >= 3) {
                gimbal_cmd = std::make_tuple(cmd_vec[0], cmd_vec[1], static_cast<bool>(cmd_vec[2]));
            } else {
                gimbal_cmd = std::make_tuple(0.0, 0.0, false);
            }

            // 数据拷贝：提取非阻塞式渲染快照
            std::lock_guard<std::mutex> r_lock(render_lock_);
            render_snapshot_ = get_render_snapshot();
        }

        // 3. 构建并发布电控/微控制器需要的最终云台控制包
        rm_interfaces::msg::GimbalControl gb_msg;
        gb_msg.header.stamp = ros_clock;
        gb_msg.header.frame_id = "tracking_frame";
        gb_msg.yaw = std::get<0>(gimbal_cmd);
        gb_msg.pitch = std::get<1>(gimbal_cmd);
        // 显式规避 bool 隐式转换风险
        gb_msg.can_fire = static_cast<int>(std::get<2>(gimbal_cmd));
        pub_gimbal_control_->publish(gb_msg);
    }
}

// =====================================================================
// 图像渲染流与可视化发布
// =====================================================================
void RmTrackerNode::res_img_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!display_) return;

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = now - last_render_time_;
    // 强制限帧到 30FPS，不浪费计算算力在显示器刷新率上限之上
    if (display_fps_limit_ && diff.count() < (1.0 / 30.0)) return;
    last_render_time_ = now;

    RenderSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(render_lock_);
        if (!render_snapshot_.has_value()) return;
        snapshot = render_snapshot_.value();
    }

    try {
        // 利用 cv_bridge 快速实现零拷贝/浅拷贝数据转换
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat draw_state = cv_ptr->image.clone();
        cv::Mat draw_hud = cv_ptr->image.clone();

        if (tf_.has_camera_info()) {
            draw_tracking_state(draw_state, snapshot);
            draw_aiming_hud(draw_hud, snapshot);
        }

        // 封装为 ROS 2 格式打包发回
        auto msg_state = cv_bridge::CvImage(msg->header, "bgr8", draw_state).toImageMsg();
        auto msg_hud = cv_bridge::CvImage(msg->header, "bgr8", draw_hud).toImageMsg();

        pub_tracking_state_img_->publish(*msg_state);
        pub_ballistic_img_->publish(*msg_hud);

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge 渲染管道捕获到异常: %s", e.what());
    }
}

RenderSnapshot RmTrackerNode::get_render_snapshot() {
    RenderSnapshot s;
    s.tracker_state = tracker_.tracker_state;
    // 分离无锁数据
    s.gimbal_control = render_snapshot_ ? render_snapshot_->gimbal_control : std::make_tuple(0.0, 0.0, false);
    return s;
}

// =====================================================================
// 决策颜色跟随及弹速动态同步
// =====================================================================
void RmTrackerNode::cb_opponent_color(const rm_interfaces::msg::Decision::SharedPtr msg) {
    if (follow_decision_) {
        if (target_color_ != msg->color) {
            target_color_ = msg->color;
            tracker_.target_color = msg->color;

            std::lock_guard<std::mutex> lock(tracker_lock_);
            tracker_.tracker_state = TrackerState::LOST;
            tracker_.tracked_id = -1;
            RCLCPP_INFO(this->get_logger(), "收到中央裁判系统下发决策，追踪颜色已重置切换为: %d", target_color_);
        }
    }
    // 弹速实时更新逻辑：若场上发生热枪管衰减或突发切射速，直接底层打靶数值积分器无缝同步
    if (msg->bullet_speed > 10.0) {
        if (std::abs(msg->bullet_speed - tracker_.bullet_speed) > 0.1) {
            std::lock_guard<std::mutex> lock(tracker_lock_);
            tracker_.bullet_speed = msg->bullet_speed;
        }
    }
}

// =====================================================================
// 动态参数配置修改
// =====================================================================
rcl_interfaces::msg::SetParametersResult RmTrackerNode::param_cb(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    std::lock_guard<std::mutex> lock(tracker_lock_);
    for (const auto& param : params) {
        const std::string& name = param.get_name();
        if (name == "target_color") {
            target_color_ = param.as_int();
            tracker_.target_color = target_color_;
            tracker_.tracker_state = TrackerState::LOST;
            tracker_.tracked_id = -1;
        } else if (name == "bullet_speed") {
            tracker_.bullet_speed = param.as_double();
        } else if (name == "display") {
            display_ = param.as_bool();
        }
    }
    return result;
}

// =====================================================================
// 状态标记图绘制实现
// =====================================================================
void RmTrackerNode::draw_tracking_state(cv::Mat& draw, const RenderSnapshot& snapshot) {
    if (snapshot.tracker_state == TrackerState::LOST) return;

    Eigen::Vector3d center_world(0.0, 0.0, 0.0);
    Eigen::Vector3d cam_c = tf_.world_to_cam(center_world, imu_rpy_);
    auto [uv_c, vis_c] = tf_.project_point(cam_c);
    if (vis_c) {
        cv::drawMarker(draw, uv_c, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 20, 2);
    }
}

// =====================================================================
// 枪口弹道 HUD 视图绘制实现
// =====================================================================
void RmTrackerNode::draw_aiming_hud(cv::Mat& draw, const RenderSnapshot& snapshot) {
    int h = draw.rows;
    int w = draw.cols;
    int cx = w / 2, cy = h / 2, size = 20;
    cv::Scalar pink(255, 0, 255);
    cv::line(draw, cv::Point(cx - size, cy), cv::Point(cx + size, cy), pink, 2);
    cv::line(draw, cv::Point(cx, cy - size), cv::Point(cx, cy + size), pink, 2);

    if (snapshot.tracker_state == TrackerState::LOST) {
        cv::putText(draw, "SEARCHING...", cv::Point(w / 2 - 80, h / 2 - 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(150, 150, 150), 2);
        return;
    }

    char text[128];
    bool can_fire = std::get<2>(snapshot.gimbal_control);
    cv::Scalar fire_color = can_fire ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

    std::snprintf(text, sizeof(text), "YAW: %.2f PITCH: %.2f", std::get<0>(snapshot.gimbal_control), std::get<1>(snapshot.gimbal_control));
    cv::putText(draw, text, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(draw, can_fire ? "FIRE ENABLE" : "HOLD FIRE", cv::Point(20, 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, fire_color, 2);
}

} // namespace dt46_vision

// =====================================================================
// ROS 2 Humble 节点主入口
// =====================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    // 强制开启高级多线程执行器：使接收底层底层通信与我们的算法计算真正物理并行并发，降延迟
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<dt46_vision::RmTrackerNode>(options);

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
