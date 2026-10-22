#include "rm_tracker/node.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstdint>

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
    nav_visual_display_ = this->declare_parameter("nav_visual_display", false);
    follow_decision_ = this->declare_parameter("follow_decision", false);
    display_fps_limit_ = this->declare_parameter("display_fps_limit", true);
    text_size_ = this->declare_parameter("text_size", 1.0);

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
    tracker_.ekf_QR_params.q_xyz = this->declare_parameter("ekf_QR_q_xyz", 2.5);
    tracker_.ekf_QR_params.q_yaw = this->declare_parameter("ekf_QR_q_yaw", 15.0);
    tracker_.ekf_QR_params.q_r = this->declare_parameter("ekf_QR_q_r", 0.05);
    tracker_.ekf_QR_params.r_xyz_factor = this->declare_parameter("ekf_QR_r_xyz_factor", 0.05);
    tracker_.ekf_QR_params.r_yaw = this->declare_parameter("ekf_QR_r_yaw", 0.02);
    tracker_.ekf_QR_params.stable_dist = this->declare_parameter("ekf_QR_stable_dist", 1.5);

    // 2.8 加载半径限幅参数
    tracker_.radius_params.r_max = this->declare_parameter("radius_r_max", 0.4);
    tracker_.radius_params.r_min = this->declare_parameter("radius_r_min", 0.12);

    // 2.9 加载弹道与射击参数
    tracker_.system_delay = this->declare_parameter("system_delay", 0.1);
    tracker_.controller_delay_ = this->declare_parameter("controller_delay", 0.02);
    tracker_.k_v2 = this->declare_parameter("k_v2", 0.019);
    tracker_.shootable_dist = this->declare_parameter("shootable_dist", 3.0);
    tracker_.yaw_tolerance_deg = this->declare_parameter("yaw_tolerance_deg", 5.0);
    tracker_.pitch_tolerance_deg = this->declare_parameter("pitch_tolerance_deg", 2.0);

    // 2.10 加载小陀螺检测参数
    tracker_.min_spinning_vel_ = this->declare_parameter("min_spinning_vel", 5.0);
    tracker_.spin_enter_time_ = this->declare_parameter("spin_enter_time", 0.3);
    tracker_.spin_exit_time_ = this->declare_parameter("spin_exit_time", 0.15);

    // 2.11 重新构建弹道查找表 (基于加载的弹速和k_v2)
    tracker_.build_ballistic_lut();

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
    pub_enemy_nav_img_ = this->create_publisher<sensor_msgs::msg::Image>("/tracker/enemy_nav_img", sensor_qos);
    pub_gimbal_control_ = this->create_publisher<rm_interfaces::msg::GimbalControl>("/tracker/gimbal_control", sensor_qos);
    pub_enemy_datas_ = this->create_publisher<rm_interfaces::msg::EnemyCenter>("/tracker/enemy_datas", sensor_qos);

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
    imu_buffer_.push_back({msg->header.stamp, imu_rpy_});
    while (imu_buffer_.size() > IMU_BUFFER_MAX) {
        imu_buffer_.pop_front();
    }
    imu_initialized_ = true;
}

// 取离图像时间戳最近的 IMU 样本；调用方必须持有 imu_lock_
bool RmTrackerNode::nearest_imu(const rclcpp::Time& stamp, Eigen::Vector3d& rpy, double& offset_ms) const {
    if (imu_buffer_.empty()) return false;

    auto best = imu_buffer_.begin();
    int64_t best_dt = std::llabs((best->stamp - stamp).nanoseconds());
    for (auto it = imu_buffer_.begin(); it != imu_buffer_.end(); ++it) {
        int64_t dt = std::llabs((it->stamp - stamp).nanoseconds());
        if (dt < best_dt) {
            best_dt = dt;
            best = it;
        }
    }
    rpy = best->rpy;
    offset_ms = static_cast<double>((best->stamp - stamp).nanoseconds()) / 1e6;
    return true;
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
    if (!nearest_imu(msg->header.stamp, data.imu_rpy, data.imu_stamp_offset_ms)) {
        data.imu_rpy = imu_rpy_; // 缓存还没建立时退回最新一帧
    }

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

        // 1. 将 ROS 自定义检测消息反序列化解析为我们的标准 Armor 结构体
        std::vector<Armor> raw_armors;
        // 预分配器
        raw_armors.reserve(data.msg->armors.size());

        for (const auto& a : data.msg->armors) {
            // 1. 颜色过滤：只保留目标敌人颜色的装甲板
            // target_color_ == 0 表示要打红色 (id 1~5), target_color_ == 1 表示要打蓝色 (id 6+)
            if (target_color_ == 0 && a.armor_id < 6) continue;
            if (target_color_ == 1 && a.armor_id >= 6) continue;

            // 2. 坐标系转换 (Cam -> World)
            Eigen::Vector3d raw_pos(a.dx / 1000.0, a.dy / 1000.0, a.dz / 1000.0);
            Eigen::Vector3d world_pos = tf_.cam_to_world(raw_pos, data.imu_rpy);

            // 角度对齐与转换 (Degree -> Radian)。PnP yaw 取反后再加云台 yaw。
            double gimbal_yaw_deg = data.imu_rpy(2);
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

        // 防抖保护：时间戳异常/零间隔用 10ms 兜底；真实帧间隔直接用，
        // 只有断流超过 100ms（比如丢包、切场景）才截断，避免把 40ms 的真实间隔当成 10ms。
        if (dt <= 0.001) {
            dt = 0.01;
        } else if (dt > 0.1) {
            dt = 0.1;
        }


        // 2. 线程同步锁：开始调用核心预测状态机与滤波器
        std::tuple<double, double, bool> gimbal_cmd;
        std::vector<std::pair<std::string, std::string>> tracker_logs;
        {
            std::lock_guard<std::mutex> lock(tracker_lock_);
            process_counter_++;
            imu_offset_acc_ms_ += data.imu_stamp_offset_ms;
            imu_offset_count_++;

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

        double raw_yaw = std::get<0>(gimbal_cmd);
        double raw_pitch = std::get<1>(gimbal_cmd);
        bool can_fire = static_cast<bool>(std::get<2>(gimbal_cmd));

        if (!smooth_ready_) {
            smooth_yaw_ = raw_yaw;
            smooth_pitch_ = raw_pitch;
            smooth_ready_ = true;
        } else {
            smooth_yaw_   = 0.25 * raw_yaw   + 0.75 * smooth_yaw_;
            smooth_pitch_ = 0.25 * raw_pitch + 0.75 * smooth_pitch_;
        }
        if (!can_fire) smooth_ready_ = false;

        gb_msg.yaw = smooth_yaw_;
        gb_msg.pitch = smooth_pitch_;
        // 显式规避 bool 隐式转换风险
        gb_msg.can_fire = static_cast<int>(can_fire);
        pub_gimbal_control_->publish(gb_msg);

        // 4. 发布导航方所需的敌人中心数据 (相机安装系，与 TF 父帧 camera 一致)
        //    数值与 enemy_nav_img 面板同源；LOST 约定为全 0 + armor_id=-1。
        rm_interfaces::msg::EnemyCenter ec_msg;
        ec_msg.header.stamp = ros_clock;
        ec_msg.header.frame_id = "camera";

        RenderSnapshot nav_snapshot;
        bool has_nav_snapshot = false;
        {
            std::lock_guard<std::mutex> r_lock(render_lock_);
            if (render_snapshot_.has_value()) {
                nav_snapshot = render_snapshot_.value();
                has_nav_snapshot = true;
            }
        }

        bool nav_tracked = has_nav_snapshot && nav_snapshot.tracker_state != TrackerState::LOST;
        if (nav_tracked) {
            Eigen::Vector3d world_pos(nav_snapshot.target_state(0),
                                      nav_snapshot.target_state(2),
                                      nav_snapshot.target_state(4));
            Eigen::Vector3d cam_pos = tf_.world_to_mount(world_pos, data.imu_rpy);

            // enemy_yaw：世界系方向向量转到安装系后再取 atan2
            double yaw_world = nav_snapshot.target_state(6);
            Eigen::Vector3d dir_world(std::cos(yaw_world), std::sin(yaw_world), 0.0);
            Eigen::Vector3d dir_cam = tf_.world_to_mount(dir_world, data.imu_rpy);

            ec_msg.tracked = true;
            ec_msg.yaw = static_cast<float>(gb_msg.yaw);
            ec_msg.x = static_cast<float>(cam_pos(0));
            ec_msg.y = static_cast<float>(cam_pos(1));
            ec_msg.z = static_cast<float>(cam_pos(2));
            ec_msg.enemy_yaw = static_cast<float>(std::atan2(dir_cam(1), dir_cam(0)));
            ec_msg.armor_id = nav_snapshot.target.has_value() ? nav_snapshot.target->id : -1;
        } else {
            ec_msg.tracked = false;
            ec_msg.yaw = 0.0f;
            ec_msg.x = 0.0f;
            ec_msg.y = 0.0f;
            ec_msg.z = 0.0f;
            ec_msg.enemy_yaw = 0.0f;
            ec_msg.armor_id = -1;
        }
        pub_enemy_datas_->publish(ec_msg);

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

            // 3. 打印当前云台解算状态；IMU offset = 图像时间戳与所选 IMU 样本的时间差
            double imu_offset_ms = imu_offset_count_ > 0 ? imu_offset_acc_ms_ / imu_offset_count_ : 0.0;
            imu_offset_acc_ms_ = 0.0;
            imu_offset_count_ = 0;
            int spin_flag = (render_snapshot_.has_value() && render_snapshot_->spin) ? 1 : 0;
            double vyaw = render_snapshot_.has_value() ? render_snapshot_->ekf_yaw_vel : 0.0;
            // 车心与半径打出来，方便在车上核对模型位置：ctr 到实测板的距离应≈r
            double ctr_x = 0.0, ctr_y = 0.0, ctr_r = 0.0;
            if (render_snapshot_.has_value()) {
                ctr_x = render_snapshot_->target_state(0);
                ctr_y = render_snapshot_->target_state(2);
                ctr_r = render_snapshot_->target_state(8);
            }
            RCLCPP_INFO(this->get_logger(),
                "[Tracker] FPS: %.1f | IMU offset: %.2f ms | spin: %d vyaw: %.2f | ctr: (%.2f, %.2f) r: %.3f | Gimbal - yaw: %.2f, pitch: %.2f, fire: %d",
                current_fps, imu_offset_ms, spin_flag, vyaw, ctr_x, ctr_y, ctr_r, gb_msg.yaw, gb_msg.pitch, gb_msg.can_fire);
        }
    }
}


// =====================================================================
// 图像渲染流与可视化发布
// =====================================================================
void RmTrackerNode::res_img_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!display_ && !nav_visual_display_) return;

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = now - last_render_time_;
    if (display_fps_limit_ && diff.count() < (1.0 / 30.0)) return;
    last_render_time_ = now;

    RenderSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(render_lock_);
        if (!render_snapshot_.has_value()) return;
        snapshot = render_snapshot_.value();
    }

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        if (!tf_.has_camera_info()) return;

        cv::Mat tracking_img = cv_ptr->image.clone();
        draw_tracking_state(tracking_img, snapshot);
        auto tracking_msg = cv_bridge::CvImage(msg->header, "bgr8", tracking_img).toImageMsg();
        pub_tracking_state_img_->publish(*tracking_msg);

        cv::Mat ballistic_img = cv_ptr->image.clone();
        draw_aiming_hud(ballistic_img, snapshot);
        auto ballistic_msg = cv_bridge::CvImage(msg->header, "bgr8", ballistic_img).toImageMsg();
        pub_ballistic_img_->publish(*ballistic_msg);

        // 导航可视化独立开关：与 display 互不影响
        if (nav_visual_display_) {
            cv::Mat enemy_nav_img = cv_ptr->image.clone();
            draw_enemy_nav(enemy_nav_img, snapshot);
            auto enemy_nav_msg = cv_bridge::CvImage(msg->header, "bgr8", enemy_nav_img).toImageMsg();
            pub_enemy_nav_img_->publish(*enemy_nav_msg);
        }

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
            tracker_.build_ballistic_lut();
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
            tracker_.build_ballistic_lut(); // 确保弹速更新和重建 LUT 在一起
        } else if (name == "display") {
            display_ = param.as_bool();
        } else if (name == "text_size") {
            text_size_ = param.as_double();
        } else if (name == "nav_visual_display") {
            nav_visual_display_ = param.as_bool();
        }
    }
    return result;
}

// =====================================================================
// 状态标记图绘制实现
// =====================================================================
void RmTrackerNode::draw_tracking_state(cv::Mat& draw, const RenderSnapshot& snapshot) {
    if (snapshot.tracker_state == TrackerState::LOST && snapshot.debug_yaw_armors.empty()) return;

    double scale = text_size_;

    Eigen::Vector3d current_imu;
    {
        std::lock_guard<std::mutex> lock(imu_lock_);
        current_imu = imu_rpy_;
    }

    if (!snapshot.debug_yaw_armors.empty()) {
        double font_scale = 1.0 * scale;
        int thickness = std::max(1, static_cast<int>(3 * scale));
        int stroke_thickness = thickness + std::max(1, static_cast<int>(5 * scale));

        for (const auto& armor : snapshot.debug_yaw_armors) {
            Eigen::Vector3d cam_pos = tf_.world_to_cam(armor.pos, current_imu);
            auto [uv, visible] = tf_.project_point(cam_pos);
            if (!visible) continue;

            char buf[16];
            std::snprintf(buf, sizeof(buf), "%.1f", armor.yaw);
            std::string text(buf);

            cv::Point uv_int(static_cast<int>(uv.x), static_cast<int>(uv.y));
            cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, nullptr);
            cv::Point text_org(uv_int.x - text_size.width / 2, uv_int.y + text_size.height / 2);

            cv::putText(draw, text, text_org, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        cv::Scalar(0, 0, 0), stroke_thickness);
            cv::putText(draw, text, text_org, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        cv::Scalar(255, 0, 255), thickness);
        }
    }

}

void RmTrackerNode::draw_aiming_hud(cv::Mat& draw, const RenderSnapshot& snapshot) {
    int h = draw.rows;
    int w = draw.cols;

    cv::Scalar pink(255, 0, 255);
    {
        int size = 20;
        int cx = w / 2, cy = h / 2;
        cv::line(draw, cv::Point(cx - size, cy), cv::Point(cx + size, cy), pink, 2);
        cv::line(draw, cv::Point(cx, cy - size), cv::Point(cx, cy + size), pink, 2);
    }

    if (snapshot.tracker_state == TrackerState::LOST) {
        cv::putText(draw, "SEARCHING...", cv::Point(w / 2 - 80, h / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(150, 150, 150), 2);
        return;
    }

    if (!snapshot.target.has_value()) {
        return;
    }

    Eigen::Vector3d current_imu;
    {
        std::lock_guard<std::mutex> lock(imu_lock_);
        current_imu = imu_rpy_;
    }

    Eigen::Vector3d target_cam = tf_.world_to_cam(snapshot.target.value().pos, current_imu);
    auto [uv_tgt, vis_tgt] = tf_.project_point(target_cam);
    if (vis_tgt) {
        cv::circle(draw, uv_tgt, 18, cv::Scalar(0, 0, 255), 2);
    }

    if (snapshot.muzzle_target.has_value()) {
        Eigen::Vector3d aim_cam = tf_.world_to_cam(snapshot.muzzle_target.value().pos, current_imu);
        auto [uv_aim, vis_aim] = tf_.project_point(aim_cam);
        if (vis_aim) {
            cv::drawMarker(draw, uv_aim, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 25, 2);
        }
    }

    double scale = text_size_;
    int start_x = static_cast<int>(20 * scale);
    int base_y = static_cast<int>(50 * scale);
    int step_y = static_cast<int>(30 * scale);

    double font_normal = 0.7 * scale;
    double font_large = 0.8 * scale;
    int thick_normal = std::max(1, static_cast<int>(1 * scale));
    int thick_bold = std::max(1, static_cast<int>(2 * scale));

    double dist = snapshot.target.value().dist;
    double angle_diff_deg = snapshot.target.value().angle_diff * (180.0 / M_PI);
    double yaw_tol = snapshot.yaw_tolerance_deg;
    double pitch_tol = snapshot.pitch_tolerance_deg;
    double ekf_yaw_vel = snapshot.ekf_yaw_vel;

    bool can_fire = std::get<2>(snapshot.gimbal_control);
    double gc_yaw = std::get<0>(snapshot.gimbal_control);
    double gc_pitch = std::get<1>(snapshot.gimbal_control);

    bool is_spin = snapshot.spin;
    double bullet_speed = snapshot.bullet_speed;

    cv::Scalar color_fire = can_fire ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    std::string text_fire = can_fire ? "FIRE ENABLE" : "HOLD FIRE";
    cv::Scalar color_spin = is_spin ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    std::string text_spin = is_spin ? "SPIN" : "NORMAL";

    auto put_text = [&](int row, const std::string& text, double font_s, cv::Scalar color, int thick) {
        cv::putText(draw, text, cv::Point(start_x, base_y + step_y * row),
                    cv::FONT_HERSHEY_SIMPLEX, font_s, color, thick);
    };

    char buf[128];
    int row = 0;

    put_text(row, "[" + text_spin + "]", font_large, color_spin, thick_bold); row++;

    std::snprintf(buf, sizeof(buf), "Armor w: %5.2f rad/s", ekf_yaw_vel);
    put_text(row, buf, font_normal, color_spin, thick_bold); row++;

    std::snprintf(buf, sizeof(buf), "Dist   : %5.2f m", dist);
    put_text(row, buf, font_normal, cv::Scalar(255, 255, 255), thick_normal); row++;

    std::snprintf(buf, sizeof(buf), "Diff   : %5.1f deg", angle_diff_deg);
    put_text(row, buf, font_normal, cv::Scalar(255, 255, 255), thick_normal); row++;

    std::snprintf(buf, sizeof(buf), "Pitch  : %5.2f deg", gc_pitch);
    put_text(row, buf, font_normal, cv::Scalar(255, 255, 255), thick_normal); row++;

    std::snprintf(buf, sizeof(buf), "Yaw    : %5.2f deg", gc_yaw);
    put_text(row, buf, font_normal, cv::Scalar(255, 255, 255), thick_normal); row++;

    std::snprintf(buf, sizeof(buf), "P_Tol  : %5.2f deg", pitch_tol);
    put_text(row, buf, font_normal, cv::Scalar(255, 255, 255), thick_normal); row++;

    std::snprintf(buf, sizeof(buf), "Y_Tol  : %5.2f deg", yaw_tol);
    put_text(row, buf, font_normal, cv::Scalar(255, 255, 255), thick_normal); row++;

    std::snprintf(buf, sizeof(buf), "Bullet Speed: %5.2f m/s", bullet_speed);
    put_text(row, buf, font_normal, cv::Scalar(255, 255, 255), thick_normal); row++;

    put_text(row, "[" + text_fire + "]", font_large, color_fire, thick_bold);
}

// =====================================================================
// 导航可视化：整车结构 + 左上角 EnemyCenter 同源数据面板
// =====================================================================
void RmTrackerNode::draw_enemy_nav(cv::Mat& draw, const RenderSnapshot& snapshot) {
    double scale = text_size_;

    Eigen::Vector3d current_imu;
    {
        std::lock_guard<std::mutex> lock(imu_lock_);
        current_imu = imu_rpy_;
    }

    // ---------- 第一层：按 EKF 状态绘制整车结构 ----------
    if (snapshot.tracker_state != TrackerState::LOST) {
        int circle_r = std::max(2, static_cast<int>(5 * scale));
        int marker_size = static_cast<int>(20 * scale);
        int thick_1 = std::max(1, static_cast<int>(1 * scale));
        int thick_2 = std::max(1, static_cast<int>(2 * scale));

        double xc = snapshot.target_state(0);
        double yc = snapshot.target_state(2);
        double za = snapshot.target_state(4);
        double yaw = snapshot.target_state(6);
        double r1 = snapshot.target_state(8);
        double r2 = snapshot.another_r;
        double dz = snapshot.dz;

        std::vector<std::optional<cv::Point>> armor_pts(4);

        for (int i = 0; i < 4; ++i) {
            double theta = yaw - i * (M_PI / 2.0);
            double r = (i % 2 == 0) ? r1 : r2;
            double z = (i % 2 == 0) ? za : (za + dz);
            Eigen::Vector3d world_pos(xc - r * std::cos(theta), yc - r * std::sin(theta), z);

            Eigen::Vector3d cam_pos = tf_.world_to_cam(world_pos, current_imu);
            auto [uv, visible] = tf_.project_point(cam_pos);
            if (visible) {
                cv::Point pt(static_cast<int>(uv.x), static_cast<int>(uv.y));
                cv::Scalar color = (i == 0) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
                cv::circle(draw, pt, circle_r, color, -1);
                armor_pts[i] = pt;
            }
        }

        Eigen::Vector3d center_world(xc, yc, za + dz / 2.0);
        Eigen::Vector3d cam_c = tf_.world_to_cam(center_world, current_imu);
        auto [uv_c, vis_c] = tf_.project_point(cam_c);
        if (vis_c) {
            cv::Point c_int(static_cast<int>(uv_c.x), static_cast<int>(uv_c.y));
            cv::drawMarker(draw, c_int, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, marker_size, thick_2);

            cv::Scalar line_color(100, 100, 100);
            if (armor_pts[0].has_value() && armor_pts[2].has_value()) {
                cv::line(draw, *armor_pts[0], *armor_pts[2], line_color, thick_1);
            }
            if (armor_pts[1].has_value() && armor_pts[3].has_value()) {
                cv::line(draw, *armor_pts[1], *armor_pts[3], line_color, thick_1);
            }

            double vx = snapshot.target_state(1);
            double vy = snapshot.target_state(3);
            if (std::hypot(vx, vy) > 0.1) {
                Eigen::Vector3d end_world(xc + vx * 0.5, yc + vy * 0.5, za);
                Eigen::Vector3d cam_end = tf_.world_to_cam(end_world, current_imu);
                auto [uv_end, vis_end] = tf_.project_point(cam_end);
                if (vis_end) {
                    cv::Point e_int(static_cast<int>(uv_end.x), static_cast<int>(uv_end.y));
                    cv::arrowedLine(draw, c_int, e_int, cv::Scalar(0, 0, 255), thick_2);
                }
            }
        }
    }

    // ---------- 第二层：左上角数据面板 ----------
    bool tracked = snapshot.tracker_state != TrackerState::LOST;
    double nav_yaw = 0.0, nav_x = 0.0, nav_y = 0.0, nav_z = 0.0, nav_enemy_yaw = 0.0;
    int nav_armor_id = -1;
    if (tracked) {
        // 面板展示世界系原始状态，与 nav_interface Python 版一致
        nav_yaw = std::get<0>(snapshot.gimbal_control);
        nav_x = snapshot.target_state(0);
        nav_y = snapshot.target_state(2);
        nav_z = snapshot.target_state(4);
        nav_enemy_yaw = snapshot.target_state(6);
        nav_armor_id = snapshot.target.has_value() ? snapshot.target->id : -1;
    }

    double font_scale = 0.7 * scale;
    int thick = std::max(1, static_cast<int>(1 * scale));
    int start_x = static_cast<int>(20 * scale);
    int base_y = static_cast<int>(50 * scale);
    int step_y = static_cast<int>(30 * scale);

    cv::Scalar color_tracked = tracked ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    std::vector<std::pair<std::string, cv::Scalar>> lines;
    char buf[128];

    std::snprintf(buf, sizeof(buf), "[NAV] tracked: %s", tracked ? "True" : "False");
    lines.emplace_back(buf, color_tracked);
    std::snprintf(buf, sizeof(buf), "yaw      : %6.2f deg", nav_yaw);
    lines.emplace_back(buf, cv::Scalar(255, 255, 255));
    std::snprintf(buf, sizeof(buf), "x        : %6.3f m", nav_x);
    lines.emplace_back(buf, cv::Scalar(255, 255, 255));
    std::snprintf(buf, sizeof(buf), "y        : %6.3f m", nav_y);
    lines.emplace_back(buf, cv::Scalar(255, 255, 255));
    std::snprintf(buf, sizeof(buf), "z        : %6.3f m", nav_z);
    lines.emplace_back(buf, cv::Scalar(255, 255, 255));
    std::snprintf(buf, sizeof(buf), "enemy_yaw: %6.2f rad", nav_enemy_yaw);
    lines.emplace_back(buf, cv::Scalar(255, 255, 255));
    std::snprintf(buf, sizeof(buf), "armor_id : %d", nav_armor_id);
    lines.emplace_back(buf, cv::Scalar(255, 255, 255));

    for (std::size_t i = 0; i < lines.size(); ++i) {
        cv::putText(draw, lines[i].first,
                    cv::Point(start_x, base_y + static_cast<int>(i) * step_y),
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, lines[i].second, thick);
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
