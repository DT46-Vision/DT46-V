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

    // 2.5 加载相机到枪口外参
    tracker_.cam_to_gun_pos = Eigen::Vector3d(
        this->declare_parameter("cam_to_gun_pos_x", 0.0),
        this->declare_parameter("cam_to_gun_pos_y", 0.125),
        this->declare_parameter("cam_to_gun_pos_z", 0.0));
    tracker_.cam_to_gun_rpy = Eigen::Vector3d(
        this->declare_parameter("cam_to_gun_rpy_r", 0.0),
        this->declare_parameter("cam_to_gun_rpy_p", 0.0),
        this->declare_parameter("cam_to_gun_rpy_y", 0.0));

    // 2.6 加载跟踪匹配与状态机参数
    tracker_.max_match_distance = this->declare_parameter("max_match_distance", 0.2);
    tracker_.max_match_yaw_diff = this->declare_parameter("max_match_yaw_diff", 57.0) * (M_PI / 180.0);
    tracker_.jump_cooldown_max = this->declare_parameter("jump_cooldown_max", 20);
    tracker_.tracking_thres = this->declare_parameter("tracking_thres", 5);
    tracker_.lost_thres = this->declare_parameter("lost_thres", 20);
    tracker_.dist_tol = this->declare_parameter("dist_tol", 0.15);

    // 2.7 加载 EKF Q/R 噪声参数
    tracker_.ekf_QR_params.q_xyz = this->declare_parameter("ekf_QR_q_xyz", 20.0);
    tracker_.ekf_QR_params.q_yaw = this->declare_parameter("ekf_QR_q_yaw", 100.0);
    tracker_.ekf_QR_params.q_r = this->declare_parameter("ekf_QR_q_r", 800.0);
    tracker_.ekf_QR_params.r_xyz_factor = this->declare_parameter("ekf_QR_r_xyz_factor", 0.05);
    tracker_.ekf_QR_params.r_yaw = this->declare_parameter("ekf_QR_r_yaw", 0.02);
    tracker_.ekf_QR_params.stable_dist = this->declare_parameter("ekf_QR_stable_dist", 1.5);

    // 2.8 加载半径限幅参数
    tracker_.radius_params.r_max = this->declare_parameter("radius_r_max", 0.4);
    tracker_.radius_params.r_min = this->declare_parameter("radius_r_min", 0.12);

    // 2.9 加载弹道与射击参数
    tracker_.system_delay = this->declare_parameter("system_delay", 0.1);
    tracker_.shootable_dist = this->declare_parameter("shootable_dist", 3.0);
    tracker_.yaw_tolerance_deg = this->declare_parameter("yaw_tolerance_deg", 5.0);
    tracker_.pitch_tolerance_deg = this->declare_parameter("pitch_tolerance_deg", 2.0);

    // 3. 初始化日志节流器和计时器

    logger_throttler_ = std::make_unique<LogThrottler>(this->get_logger(), 1000);
    last_fps_log_time_ = std::chrono::steady_clock::now();
    last_render_time_ = std::chrono::steady_clock::now();

    // 4. 配置通信服务质量 (QoS)：使用最适合车载传感器和高频图像的 SensorData 策略（低延迟、允许丢包）
    auto sensor_qos = rclcpp::SensorDataQoS();

    // 5. 绑定并订阅高频基础数据
    sub_imu_rpy_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/imu/rpy", 10, std::bind(&RmTrackerNode::imu_rpy_cb, this, std::placeholders::_1));

    sub_armors_ = this->create_subscription<rm_interfaces::msg::ArmorsMsg>(
        "/detector/armors_info", sensor_qos, std::bind(&RmTrackerNode::armors_cb, this, std::placeholders::_1));

    sub_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", sensor_qos, std::bind(&RmTrackerNode::camera_info_cb, this, std::placeholders::_1));

    sub_opponent_color_ = this->create_subscription<rm_interfaces::msg::Decision>(
        "/nav/decision", sensor_qos, std::bind(&RmTrackerNode::cb_opponent_color, this, std::placeholders::_1));

    sub_raw_img_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(), std::bind(&RmTrackerNode::res_img_cb, this, std::placeholders::_1));

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

    RCLCPP_INFO(this->get_logger(), "DT46 Tracker 节点启动成功。");
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
        std::lock_guard<std::mutex> lock(queue_lock_);
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
            std::unique_lock<std::mutex> lock(queue_lock_);
            // 阻塞直至受到条件变量唤醒
            cv_.wait(lock, [this]() { return !track_queue_.empty() || !worker_running_; });
            if (!worker_running_) break;

            data = track_queue_.front();
            track_queue_.pop();
        }

        // 【补全1】获取云台的世界系 Yaw (角度)，供下面的循环使用
        double gimbal_yaw_deg = data.imu_rpy(2);

        // 1. 将 ROS 自定义检测消息反序列化解析为我们的标准 Armor 结构体
        std::vector<Armor> raw_armors;
        // 预分配器
        raw_armors.reserve(data.msg->armors.size());

        for (const auto& a : data.msg->armors) {
            // 1. 颜色过滤：只保留目标敌人颜色的装甲板
            // target_color_ == 0 表示要打红色 (id 1~5), target_color_ == 1 表示要打蓝色 (id 6+)
            if (target_color_ == 0 && a.armor_id >= 6) continue;
            if (target_color_ == 1 && a.armor_id < 6) continue;

            // 2. 坐标系转换 (Cam -> World)
            Eigen::Vector3d raw_pos(a.dx / 1000.0, a.dy / 1000.0, a.dz / 1000.0);
            Eigen::Vector3d world_pos = tf_.cam_to_world(raw_pos, data.imu_rpy);

            // 3. 角度对齐与转换 (Degree -> Radian)
            double pnp_yaw_deg = -a.yaw;
            double world_yaw_deg = gimbal_yaw_deg + pnp_yaw_deg;
            double yaw_rad = world_yaw_deg * (M_PI / 180.0);
            double norm_yaw = std::fmod(yaw_rad + M_PI, 2.0 * M_PI);
            if (norm_yaw < 0) norm_yaw += 2.0 * M_PI;
            norm_yaw -= M_PI;

            Armor armor(a.armor_id, world_pos(0), world_pos(1), world_pos(2), norm_yaw);
            raw_armors.push_back(armor);
        }

        // 把原版调用 Tracker 进行 EKF 更新的代码加回来
        rclcpp::Time ros_clock = data.msg->header.stamp;

        // 动态获取真实 dt
        static rclcpp::Time last_time = data.msg->header.stamp;
        rclcpp::Time current_time = data.msg->header.stamp;

        double dt = (current_time - last_time).nanoseconds() / 1e9;
        last_time = current_time;

        // 防抖保护：如果时间戳异常或停顿太久，限制 dt 范围
        if (dt <= 0.001 || dt > 0.1) {
            dt = 0.01;
        }


        // 2. 线程同步锁：开始调用核心预测状态机与滤波器
        std::tuple<double, double, bool> gimbal_cmd;
        std::vector<std::pair<std::string, std::string>> tracker_logs;
        {
            std::lock_guard<std::mutex> lock(tracker_lock_);
            process_counter_++;

            // 解包复合返回值并安全隔离底层
            auto track_result = tracker_.track(tf_, raw_armors, data.imu_rpy, dt);
            auto cmd_vec = std::get<0>(track_result);

            tracker_logs = std::get<1>(track_result);

            // 严谨校验解算向量完整度，防爆越界
            if (cmd_vec.size() >= 3) {
                gimbal_cmd = std::make_tuple(cmd_vec[0], cmd_vec[1], static_cast<bool>(cmd_vec[2]));
            } else {
                gimbal_cmd = std::make_tuple(0.0, 0.0, false);
            }

            // 数据拷贝：提取非阻塞式渲染快照
            std::lock_guard<std::mutex> r_lock(render_lock_);
            render_snapshot_ = tracker_.get_render_snapshot();
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

        if (logger_throttler_->should_log()) {
            // 1. 计算 FPS
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> dt_fps = now - last_fps_log_time_;
            double current_fps = process_counter_ / dt_fps.count();
            process_counter_ = 0;
            last_fps_log_time_ = now;

            // 2. 打印底层 Tracker 传上来的状态机转换日志
            for (const auto& log : tracker_logs) {  // <--- 改成遍历 tracker_logs
                RCLCPP_INFO(this->get_logger(), "%s", log.second.c_str());
            }

            // 3. 打印当前云台解算状态
            RCLCPP_INFO(this->get_logger(),
                "[Tracker] FPS: %.1f | Gimbal - yaw: %.2f, pitch: %.2f, fire: %d",
                current_fps, gb_msg.yaw, gb_msg.pitch, gb_msg.can_fire);
        }
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

    if (snapshot.target.has_value()) {
        Eigen::Vector3d target_world = snapshot.target.value().pos; // 世界系 3D 坐标

        // 获取当前安全的 IMU 数据
        Eigen::Vector3d current_imu;
        {
            std::lock_guard<std::mutex> lock(imu_lock_);
            current_imu = imu_rpy_;
        }

        // 1. 世界系转回相机系
        Eigen::Vector3d target_cam = tf_.world_to_cam(target_world, current_imu);

        // 2. 3D 相机系坐标投影为 2D 像素坐标 (u, v)
        auto [uv, is_valid] = tf_.project_point(target_cam);

        if (is_valid) {
            // 依据距离动态计算锁定圈的大小（越远圈越小）
            int radius = static_cast<int>(35.0 / snapshot.target.value().dist);
            radius = std::clamp(radius, 15, 60);

            // 绘制锁定圈（发射允许为绿色，不允许为红色）
            cv::circle(draw, uv, radius, fire_color, 2);

            // 标签文字：展示当前追踪的 ID 和是否进入小陀螺
            std::string state_label = snapshot.spin ? " [SPIN]" : " [NORMAL]";
            std::string full_text = "ID:" + std::to_string(snapshot.target.value().id) + state_label;

            cv::putText(draw, full_text, cv::Point(uv.x + radius + 4, uv.y - radius),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, fire_color, 2);
            }
        }
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
