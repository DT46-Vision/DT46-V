#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <optional>
#include <string>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <condition_variable> // 引入

#include "rm_detector/detector.hpp"
#include "rm_detector/pnp.hpp"
#include "rm_detector/classifier.hpp"
#include "rm_interfaces/msg/armor_info.hpp"
#include "rm_interfaces/msg/armors_msg.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono;
using namespace cv;
using namespace std;

namespace DT46_VISION {

    class ArmorDetectorNode : public rclcpp::Node {

    public:
        ArmorDetectorNode() : Node("rm_detector") {
            // ---------------- 参数声明 ----------------
            // 先声明所有参数（从 detector_params.yaml 复制默认值）
            this->declare_parameter<std::string>("cls_model_file", "mlp.onnx");

            // roi 裁剪参数
            this->declare_parameter<bool>("roi_crop", false);
            this->declare_parameter<double>("roi_scale", 0.5);

            // 灯条过滤参数
            this->declare_parameter<int>("light_area_min", 5);
            this->declare_parameter<double>("light_h_w_ratio", 5.0);
            this->declare_parameter<int>("light_angle_min", -35);
            this->declare_parameter<int>("light_angle_max", 35);
            this->declare_parameter<double>("light_red_ratio", 2.0);
            this->declare_parameter<double>("light_blue_ratio", 2.0);

            // 装甲板匹配参数
            this->declare_parameter<double>("height_rate_tol", 1.3);
            this->declare_parameter<double>("height_multiplier_min", 1.8);
            this->declare_parameter<double>("height_multiplier_max", 3.0);
            this->declare_parameter<double>("row_stretch", 2.22f);
            this->declare_parameter<double>("col_stretch", 0.66f);

            // 图像处理参数
            this->declare_parameter<int>("binary_val", 120);
            this->declare_parameter<int>("detect_color", 2);
            this->declare_parameter<bool>("display_mode", false);

            // 其他参数
            this->declare_parameter<bool>("use_geometric_center", true);
            this->declare_parameter<int>("print_period_ms", 1000); // ms

            // 后置 USB 相机（仅检测有无目标，供云台转 yaw）
            this->declare_parameter<bool>("enable_rear_camera", false);   // false时为前置摄像头瞄准
            this->declare_parameter<std::string>("rear_image_topic", "/usb_camera/image_raw");   // 后置摄像头图像数据话题
            this->declare_parameter<std::string>("rear_camera_info_topic", "/usb_camera/camera_info");  // 后置摄像头参数话题
            this->declare_parameter<int>("rear_max_age_ms", 300);    // 后置摄像头图像过期阈值
            this->declare_parameter<int>("front_max_gap_ms", 500);   // 前置摄像头断流阈值
            this->declare_parameter<std::vector<double>>("rear_ext_xyz_mm", {0.0, 0.0, 0.0});    // 后相机相对前相机的平移
            this->declare_parameter<std::vector<double>>("rear_ext_rpy_deg", {0.0, 180.0, 0.0}); // 后相机相对前相机的旋转

            // ---------------- 参数读取 ----------------
            std::string cls_model_file   = get_required_param<std::string>("cls_model_file");

            // 拼接模型路径
            std::string pkg_share = ament_index_cpp::get_package_share_directory("rm_detector");
            cls_model_path_ = (std::filesystem::path(pkg_share) / "model" / cls_model_file).string();

            RCLCPP_INFO(this->get_logger(), "Classifier model file: %s", cls_model_file.c_str());
            RCLCPP_INFO(this->get_logger(), "Classifier model path: %s", cls_model_path_.c_str());

            Params params = {
                get_required_param<int>("light_area_min"),
                get_required_param<double>("light_h_w_ratio"),
                get_required_param<int>("light_angle_min"),
                get_required_param<int>("light_angle_max"),
                get_required_param<double>("light_red_ratio"),
                get_required_param<double>("light_blue_ratio"),
                get_required_param<double>("height_rate_tol"),
                get_required_param<double>("height_multiplier_min"),
                get_required_param<double>("height_multiplier_max"),
                get_required_param<double>("row_stretch"),
                get_required_param<double>("col_stretch"),
                get_required_param<bool>("roi_crop"),
                get_required_param<double>("roi_scale")
            };

            detect_color_        = get_required_param<int>("detect_color");
            display_mode_        = get_required_param<bool>("display_mode");
            binary_val_          = get_required_param<int>("binary_val");
            use_geometric_center_= get_required_param<bool>("use_geometric_center");
            print_period_ms_.store(get_required_param<int>("print_period_ms")); // 多线程访问

            // 后相机参数
            enable_rear_camera_.store(get_required_param<bool>("enable_rear_camera"));
            rear_max_age_ms_.store(get_required_param<int>("rear_max_age_ms"));
            front_max_gap_ms_.store(get_required_param<int>("front_max_gap_ms"));
            rear_image_topic_       = get_required_param<std::string>("rear_image_topic");
            rear_camera_info_topic_ = get_required_param<std::string>("rear_camera_info_topic");
            update_rear_extrinsic_(
                get_required_param<std::vector<double>>("rear_ext_xyz_mm"),
                get_required_param<std::vector<double>>("rear_ext_rpy_deg"));

            // ---------------- Detector 初始化 ----------------
            detector_ = std::make_shared<ArmorDetector>(detect_color_, display_mode_, binary_val_, params);
            pnp_      = std::make_shared<PNP>(this->get_logger());

            reload_classifier_impl_(cls_model_path_);

            // 动态参数回调
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&ArmorDetectorNode::parameters_callback, this, std::placeholders::_1));

            // ---------------- 订阅/发布 ----------------
            auto sensor_qos = rclcpp::SensorDataQoS();
            sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/image_raw", sensor_qos, std::bind(&ArmorDetectorNode::image_callback, this, std::placeholders::_1));
            // camera_info 从 RELIABLE(10) 改为 SensorDataQoS:
            sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera_info", sensor_qos, std::bind(&ArmorDetectorNode::camera_info_callback, this, std::placeholders::_1));

            // 后置相机订阅（始终创建，逻辑由 enable_rear_camera_ 参数门控）
            sub_rear_image_ = this->create_subscription<sensor_msgs::msg::Image>(
                rear_image_topic_, sensor_qos, std::bind(&ArmorDetectorNode::rear_image_callback, this, std::placeholders::_1));
            sub_rear_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                rear_camera_info_topic_, sensor_qos, std::bind(&ArmorDetectorNode::rear_camera_info_callback, this, std::placeholders::_1));

            // 【修改点】将装甲板坐标的发布策略改为 SensorDataQoS，消除底层的缓冲队列延迟
            publisher_armors_     = this->create_publisher<rm_interfaces::msg::ArmorsMsg>("/detector/armors_info", sensor_qos);
            
            // （可选优化）如果是比赛实战，建议把下面这些 debug 图像的发布也都改成 sensor_qos，防止因为图像推流挤占网卡带宽导致卡顿
            publisher_result_img_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/result", sensor_qos);
            publisher_result_img_rear_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/result_rear", sensor_qos);
            publisher_crop_img_   = this->create_publisher<sensor_msgs::msg::Image>("/detector/crop_img", sensor_qos);
            publisher_bin_img_    = this->create_publisher<sensor_msgs::msg::Image>("/detector/bin_img", sensor_qos);
            publisher_armor_img_  = this->create_publisher<sensor_msgs::msg::Image>("/detector/img_armor", sensor_qos);
            publisher_armor_processed_img_  = this->create_publisher<sensor_msgs::msg::Image>("/detector/img_armor_processed", sensor_qos);

            // 工作线程detect_color_
            running_.store(true);
            worker_ = std::thread(&ArmorDetectorNode::processing_loop, this);

            RCLCPP_INFO(this->get_logger(), "Armor Detector Node has been started.");
        }

        ~ArmorDetectorNode() override {
            running_.store(false);
            if (worker_.joinable()) worker_.join();
        }

    private:
        // ---------------- 工具函数 ----------------
        template<typename T>
        T get_required_param(const std::string& name) {
            T value;
            if (!this->get_parameter(name, value)) {
                RCLCPP_ERROR(this->get_logger(), "Required parameter '%s' not found in YAML!", name.c_str());
                throw std::runtime_error("Missing required parameter: " + name);
            }
            return value;
        }

        // ---------------- 分类器加载 ----------------
        void reload_classifier_impl_(const std::string& onnx_path) {
            if (onnx_path.empty()) {
                classifier_.reset();
                if (detector_) detector_->set_classifier(nullptr);
                RCLCPP_WARN(this->get_logger(), "[Classifier] Disabled (cls_model_file is empty).");
                return;
            }
            try {
                classifier_ = std::make_shared<NumberClassifier>(onnx_path, cv::Size(20, 28));
                if (detector_) detector_->set_classifier(classifier_);
                RCLCPP_INFO(this->get_logger(), "[Classifier] Loaded ONNX: %s",
                            onnx_path.c_str());
            } catch (const std::exception& e) {
                classifier_.reset();
                if (detector_) detector_->set_classifier(nullptr);
                RCLCPP_ERROR(this->get_logger(), "[Classifier] Load failed: %s", e.what());
            }
        }

        // 帧与该相机的内参配对缓存，arrival 用稳钟避免时间源不一致
        struct FramePacket {
            cv_bridge::CvImageConstPtr frame;
            sensor_msgs::msg::CameraInfo::SharedPtr caminfo;
            std::chrono::steady_clock::time_point arrival{};
        };

        // ---------------- 图像回调 (生产者) ----------------
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "cv_bridge toCvShare failed: %s", e.what());
                return;
            }

            {
                // 使用花括号限制锁的生命周期，赋值完立即解锁
                std::lock_guard<std::mutex> lock(frame_mtx_);
                front_.frame = cv_ptr;
                front_.arrival = std::chrono::steady_clock::now();
            }

            // [核心修改] 发出信号，唤醒正在 wait 的处理线程
            new_frame_cv_.notify_one();
        }

        // 将ros原始消息封装为opencv的格式
        void rear_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "rear cv_bridge failed: %s", e.what());
                return;
            }

            {
                std::lock_guard<std::mutex> lock(frame_mtx_);
                rear_.frame = cv_ptr;
                rear_.arrival = std::chrono::steady_clock::now();
            }

            new_frame_cv_.notify_one();
        }

        // ---------------- 相机内参回调 ----------------
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(frame_mtx_);
            front_.caminfo = msg;
        }

        void rear_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(frame_mtx_);
            rear_.caminfo = msg;
        }

        // ---------------- 后相机工具 ----------------
        // 由 RPY(度) + 平移(mm) 构建外参缓存; R = Rz(yaw)·Ry(pitch)·Rx(roll)
        // 光学系 Z前/X右/Y下，后置相机绕垂直轴(Y)转180°，故默认 pitch=180
        void update_rear_extrinsic_(const std::vector<double>& xyz_mm,
                                    const std::vector<double>& rpy_deg) {
            if (xyz_mm.size() != 3 || rpy_deg.size() != 3) {
                RCLCPP_ERROR(this->get_logger(), "rear_ext 参数必须是 3 个数");
                return;
            }
            const double d2r = CV_PI / 180.0;
            double roll = rpy_deg[0] * d2r, pitch = rpy_deg[1] * d2r, yaw = rpy_deg[2] * d2r;
            cv::Mat Rz = (cv::Mat_<double>(3,3) <<
                cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1);
            cv::Mat Ry = (cv::Mat_<double>(3,3) <<
                cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch));
            cv::Mat Rx = (cv::Mat_<double>(3,3) <<
                1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll));
            std::lock_guard<std::mutex> lock(ext_mtx_);
            R_ext_ = Rz * Ry * Rx;
            t_ext_ = (cv::Mat_<double>(3,1) << xyz_mm[0], xyz_mm[1], xyz_mm[2]);
        }

        // 前帧超过 front_max_gap_ms 无新图视为断流(<=0=永不启用后帧直连)
        bool front_stale_locked_() const {
            int gap = front_max_gap_ms_.load();
            if (gap <= 0) return false;
            auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - front_.arrival).count();
            return age > gap;
        }

        // 后帧是否可用于兜底检测 (须持 frame_mtx_)
        bool rear_usable_locked_() const {
            if (!enable_rear_camera_.load()) return false;
            if (rear_.frame == nullptr) return false;
            auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - rear_.arrival).count();
            return age <= rear_max_age_ms_.load();
        }

        // ---------------- 主处理线程 (消费者) ----------------
        void processing_loop() {
            using clock = std::chrono::steady_clock;

            // 颜色定义保持不变
            const std::string GREEN = "\033[32m";
            const std::string CYAN  = "\033[96m";
            const std::string PINK  = "\033[38;5;218m";
            const std::string RESET = "\033[0m";

            auto last_print = clock::now();

            // 周期内状态
            bool had_detection_in_period = false;
            std::vector<rm_interfaces::msg::ArmorInfo> last_detected_armors;

            // FPS 统计
            int frame_count = 0;
            auto last_fps_time = clock::now();
            double current_fps = 0.0;

            while (rclcpp::ok() && running_.load()) {

                FramePacket front_packet;   // 本轮取到的前帧(可能为空)
                FramePacket rear_packet;    // 本轮实际用于检测的后帧(可能为空)
                bool use_rear_direct = false; // 前帧断流，直接处理后帧

                // ==================== 选帧 ====================
                {
                    // 必须使用 unique_lock 配合 condition_variable
                    std::unique_lock<std::mutex> lock(frame_mtx_);

                    // 线程在此处“死等”，直到被唤醒且满足条件:
                    // 退出 / 前帧到达 / (后相机启用且前帧断流且后帧到达)
                    // 后帧也参与唤醒条件，前相机拔线时不会永远空等
                    new_frame_cv_.wait(lock, [this] {
                        if (!this->running_.load()) return true;
                        if (this->front_.frame != nullptr) return true;
                        return this->rear_usable_locked_() && this->front_stale_locked_();
                    });

                    // 如果是因为程序退出被唤醒，则跳出循环
                    if (!running_.load()) break;

                    // 前帧优先; 前帧断流才直接用后帧
                    if (front_.frame != nullptr) {
                        front_packet = front_;
                        // [关键一步] 取走数据后，把指针置空！
                        // 这确保了下一轮循环如果相机还没来新图，wait 会继续卡住，
                        // 从而彻底根除了“重复识别同一帧”的问题。
                        front_.frame = nullptr;
                    } else {
                        rear_packet = rear_;
                        rear_.frame = nullptr;
                        use_rear_direct = true;
                    }
                }
                // ==================== 选帧 End ====================

                frame_count++;

                rm_interfaces::msg::ArmorsMsg armors_msg;
                armors_msg.header.stamp = this->get_clock()->now();
                armors_msg.header.frame_id = "camera_frame";

                // 本次检测结果的来源: false=前相机, true=后相机
                bool from_rear = false;

                if (!use_rear_direct) {
                    // -------- 前相机轮 --------
                    if (front_packet.frame->image.empty()) continue;
                    cv::Mat frame = front_packet.frame->image;

                    cv::Mat crop, bin, result, img_armor, img_armor_processed;
                    std::vector<Armor> armors;
                    bool detection_error = false;
                    try {
                        armors = detector_->detect_armors(frame);
                        // 注意：display() 内部如果涉及耗时绘图，建议仅在调试时开启
                        std::tie(crop, bin, result, img_armor, img_armor_processed) = detector_->display();
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Detection error: %s", e.what());
                        detection_error = true;
                    }

                    // 前相机调试图永远只发原话题
                    if (display_mode_) {
                        if (!crop.empty())
                            publisher_crop_img_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", crop).toImageMsg());
                        if (!bin.empty())
                            publisher_bin_img_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", bin).toImageMsg());
                        if (!result.empty())
                            publisher_result_img_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result).toImageMsg());
                        if (!img_armor.empty())
                            publisher_armor_img_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_armor).toImageMsg());
                        if (!img_armor_processed.empty())
                            publisher_armor_processed_img_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img_armor_processed).toImageMsg());
                    }

                    if (!detection_error && !armors.empty()) {
                        had_detection_in_period = true;
                        last_detected_armors.clear();

                        for (const auto& armor : armors) {
                            rm_interfaces::msg::ArmorInfo armor_info;
                            armor_info.armor_id = armor.armor_id;

                            // 这里的 processArmorCorners 可能会耗时，需注意优化
                            auto [dx, dy, dz, yaw] = pnp_->processArmorCorners(
                                front_packet.caminfo, use_geometric_center_, frame, armor, armor.armor_id);

                            armor_info.dx = dx; armor_info.dy = dy; armor_info.dz = dz;
                            armor_info.yaw = yaw;
                            last_detected_armors.push_back(armor_info);
                            armors_msg.armors.push_back(armor_info);
                        }
                    } else {
                        // 优先级: 前相机无目标 → 用后相机兜底
                        std::lock_guard<std::mutex> lock(frame_mtx_);
                        if (rear_usable_locked_()) {
                            rear_packet = rear_;
                            rear_.frame = nullptr;
                        }
                    }
                }

                // -------- 后相机处理 (前帧断流直连 或 前无目标兜底) --------
                if (use_rear_direct || rear_packet.frame) {
                    if (!rear_packet.frame) continue;
                    if (rear_packet.frame->image.empty()) continue;
                    cv::Mat frame = rear_packet.frame->image;

                    cv::Mat result;
                    std::vector<Armor> armors;
                    bool detection_error = false;
                    try {
                        armors = detector_->detect_armors(frame);
                        cv::Mat crop, bin, img_armor, img_armor_processed;
                        std::tie(crop, bin, result, img_armor, img_armor_processed) = detector_->display();
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Rear detection error: %s", e.what());
                        detection_error = true;
                    }

                    // 后相机调试图只发新话题，不动现有 /detector/*
                    if (display_mode_ && !result.empty())
                        publisher_result_img_rear_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result).toImageMsg());

                    if (!detection_error && !armors.empty()) {
                        from_rear = true;
                        had_detection_in_period = true;
                        last_detected_armors.clear();

                        // 取外参缓存(参数回调可能随时重建，须持锁拷贝)
                        cv::Mat R_ext, t_ext;
                        { std::lock_guard<std::mutex> lock(ext_mtx_); R_ext = R_ext_; t_ext = t_ext_; }

                        for (const auto& armor : armors) {
                            rm_interfaces::msg::ArmorInfo armor_info;
                            armor_info.armor_id = armor.armor_id;

                            auto [dx, dy, dz, yaw] = pnp_->processArmorCorners(
                                rear_packet.caminfo, use_geometric_center_, frame, armor, armor.armor_id,
                                R_ext, t_ext);

                            armor_info.dx = dx; armor_info.dy = dy; armor_info.dz = dz;
                            armor_info.yaw = yaw;
                            last_detected_armors.push_back(armor_info);
                            armors_msg.armors.push_back(armor_info);
                        }
                    }
                }

                publisher_armors_->publish(armors_msg);

                // -------- 打印节流逻辑 (保持不变) --------
                int pp_ms = print_period_ms_.load();
                auto now = clock::now();
                if (pp_ms <= 0 ||
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() >= pp_ms) {

                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
                    if (elapsed > 0) {
                        current_fps = frame_count * 1000.0 / elapsed;
                    }
                    frame_count = 0;
                    last_fps_time = now;

                    if (had_detection_in_period) {
                        for (const auto& armor_info : last_detected_armors) {
                            RCLCPP_INFO(this->get_logger(),
                                "发布 [%s] %sID:%d%s | %sdx:%.2f%s | %sdy:%.2f%s | %sdz:%.2f%s | %syaw:%.2f%s",
                                from_rear ? "后" : "前",
                                GREEN.c_str(), armor_info.armor_id, RESET.c_str(),
                                CYAN.c_str(), armor_info.dx, RESET.c_str(),
                                CYAN.c_str(), armor_info.dy, RESET.c_str(),
                                CYAN.c_str(), armor_info.dz, RESET.c_str(),
                                CYAN.c_str(), armor_info.yaw, RESET.c_str()
                            );
                        }
                    } else {
                        RCLCPP_INFO(this->get_logger(), "%sNo armors detected%s", PINK.c_str(), RESET.c_str());
                    }

                    RCLCPP_WARN(this->get_logger(), "[FPS_detector] %.1f", current_fps);

                    had_detection_in_period = false;
                    last_detected_armors.clear();
                    last_print = now;
                }
            }
        }

        // ---------------- 动态参数回调 ----------------
        rcl_interfaces::msg::SetParametersResult
        parameters_callback(const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true; result.reason = "success";

            std::optional<std::string> new_model_file;
            bool ext_dirty = false;

            for (const auto& param : parameters) {
                const auto& name = param.get_name();
                if (name == "cls_model_file") {
                    new_model_file = param.as_string();
                } else if (name == "light_area_min") { detector_->update_light_area_min(param.as_int());
                } else if (name == "light_h_w_ratio") { detector_->update_light_h_w_ratio(param.as_double());
                } else if (name == "light_angle_min") { detector_->update_light_angle_min(param.as_int());
                } else if (name == "light_angle_max") { detector_->update_light_angle_max(param.as_int());
                } else if (name == "light_red_ratio") { detector_->update_light_red_ratio(param.as_double());
                } else if (name == "light_blue_ratio") { detector_->update_light_blue_ratio(param.as_double());
                } else if (name == "height_rate_tol") { detector_->update_height_rate_tol(param.as_double());
                } else if (name == "height_multiplier_min") { detector_->update_height_multiplier_min(param.as_double());
                } else if (name == "height_multiplier_max") { detector_->update_height_multiplier_max(param.as_double());
                } else if (name == "row_stretch") { detector_->update_row_stretch(param.as_double());
                } else if (name == "col_stretch") { detector_->update_col_stretch(param.as_double());
                } else if (name == "binary_val") { detector_->update_binary_val(param.as_int());
                } else if (name == "detect_color") { detector_->update_detect_color(param.as_int());
                } else if (name == "display_mode") { detector_->update_display_mode(param.as_bool()); display_mode_ = param.as_bool();
                } else if (name == "print_period_ms") { print_period_ms_.store(param.as_int());
                } else if (name == "roi_crop") { detector_->update_roi_crop(param.as_bool());
                } else if (name == "roi_scale") { detector_->update_roi_scale(param.as_double());
                } else if (name == "enable_rear_camera") { enable_rear_camera_.store(param.as_bool());
                } else if (name == "rear_max_age_ms") { rear_max_age_ms_.store(param.as_int());
                } else if (name == "front_max_gap_ms") { front_max_gap_ms_.store(param.as_int());
                } else if (name == "rear_ext_xyz_mm" || name == "rear_ext_rpy_deg") { ext_dirty = true;
                }
            }

            // 两组外参一起重建，避免中间态
            if (ext_dirty) {
                std::vector<double> xyz, rpy;
                this->get_parameter("rear_ext_xyz_mm", xyz);
                this->get_parameter("rear_ext_rpy_deg", rpy);
                update_rear_extrinsic_(xyz, rpy);
            }

            if (new_model_file) {
                std::string pkg_share = ament_index_cpp::get_package_share_directory("rm_detector");
                std::string new_path = cls_model_path_;
                if (new_model_file) {
                    new_path = (std::filesystem::path(pkg_share) / "model" / *new_model_file).string();
                }
                reload_classifier_impl_(new_path);
            }
            return result;
        }

    private:
        // ---- ROS 通道
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        sub_image_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr   sub_camera_info_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        sub_rear_image_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr   sub_rear_camera_info_;
        rclcpp::Publisher<rm_interfaces::msg::ArmorsMsg>::SharedPtr     publisher_armors_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           publisher_result_img_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           publisher_result_img_rear_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           publisher_armor_img_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           publisher_armor_processed_img_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           publisher_bin_img_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           publisher_crop_img_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

        // ---- 模块实例
        std::shared_ptr<ArmorDetector> detector_;
        std::shared_ptr<PNP>           pnp_;
        std::shared_ptr<NumberClassifier> classifier_;

        // ---- 参数缓存
        std::string cls_model_path_;
        int detect_color_;
        bool display_mode_;
        int binary_val_;
        bool use_geometric_center_;
        std::atomic<int> print_period_ms_{1000};
        std::condition_variable new_frame_cv_;

        // ---- 后相机参数 ----
        std::atomic<bool> enable_rear_camera_{false};
        std::atomic<int>  rear_max_age_ms_{300};
        std::atomic<int>  front_max_gap_ms_{500};
        std::string rear_image_topic_;
        std::string rear_camera_info_topic_;
        std::mutex ext_mtx_;   // 保护外参矩阵(参数回调线程重建 vs 工作线程读取)
        cv::Mat R_ext_;
        cv::Mat t_ext_;

        // ---- 缓存与线程 ----
        std::mutex frame_mtx_;
        FramePacket front_;   // 前相机帧+内参配对槽位
        FramePacket rear_;    // 后相机帧+内参配对槽位
        std::thread worker_;
        std::atomic<bool> running_{false};
    };

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DT46_VISION::ArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
