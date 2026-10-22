#include <array>
#include <chrono>
#include <mutex>
#include <numbers>
#include <optional>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "modules/hardware/protocol.hpp"
#include "modules/hardware/serial_port.hpp"

namespace {
using namespace std::chrono_literals;namespace hardware=dt46::hardware;
class DmImuNode : public rclcpp::Node {
public:
    DmImuNode():Node("dm_imu") {
        const auto path=declare_parameter<std::string>("port","/dev/ttyACM0");const int baud=declare_parameter<int>("baudrate",921600);
        frame_id=declare_parameter<std::string>("frame_id","imu_link");degrees=declare_parameter<bool>("publish_rpy_in_degree",true);verbose=declare_parameter<bool>("verbose",false);
        const bool reliable=declare_parameter<bool>("qos_reliable",false),data=declare_parameter<bool>("publish_imu_data",false),rpy=declare_parameter<bool>("publish_rpy",true),pose=declare_parameter<bool>("publish_pose",false);
        rclcpp::QoS qos=rclcpp::SensorDataQoS();if(reliable)qos=rclcpp::QoS(50).reliable().durability_volatile();
        if(data)imu_publisher=create_publisher<sensor_msgs::msg::Imu>("imu/data",qos);
        if(rpy)rpy_publisher=create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy",qos);
        if(pose)pose_publisher=create_publisher<geometry_msgs::msg::PoseStamped>("imu/pose",10);
        port.open(path,baud,0ms);RCLCPP_INFO(get_logger(),"IMU serial opened: %s @ %d",path.c_str(),baud);
        reader=std::jthread([this](std::stop_token stop){read_loop(stop);});
        publish_timer=create_wall_timer(10ms,[this]{publish_latest();});stats_timer=create_wall_timer(2s,[this]{statistics();});
    }
    ~DmImuNode()override{reader.request_stop();if(reader.joinable())reader.join();}
private:
    std::string frame_id;bool degrees=true,verbose=false;hardware::SerialPort port;
    std::mutex mutex;std::optional<hardware::ImuSample> latest;std::uint64_t sequence=0,published_sequence=0,published=0,empty_ticks=0;
    std::array<std::uint64_t,4> counters{};std::string error;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::TimerBase::SharedPtr publish_timer,stats_timer;std::jthread reader;
    void read_loop(std::stop_token stop) {
        try {
            hardware::ImuParser parser;std::array<std::uint8_t,4096> bytes{};
            while(!stop.stop_requested()&&rclcpp::ok()) {
                const auto count=port.read(bytes);const auto frames=parser.feed(std::span(bytes).first(count));
                if(count){std::lock_guard lock(mutex);if(!frames.empty()){latest=frames.back();++sequence;}counters={parser.accepted,parser.crc_failures,parser.invalid_frames,parser.discarded_bytes};}
                std::this_thread::sleep_for(1ms);
            }
        }catch(const std::exception& e){std::lock_guard lock(mutex);error=e.what();port.close();}
    }
    void publish_latest() {
        std::optional<hardware::ImuSample> sample;
        {std::lock_guard lock(mutex);if(latest&&sequence!=published_sequence){sample=latest;published_sequence=sequence;}}
        if(!sample){if(++empty_ticks%200==0&&verbose)RCLCPP_WARN(get_logger(),"No new IMU frames for approximately 2 seconds");return;}empty_ticks=0;
        const auto stamp=now();const auto& rpy=sample->rpy;const auto q=hardware::quaternion(rpy);
        geometry_msgs::msg::Quaternion orientation;orientation.x=q[0];orientation.y=q[1];orientation.z=q[2];orientation.w=q[3];
        if(rpy_publisher){geometry_msgs::msg::Vector3Stamped message;message.header.stamp=stamp;message.header.frame_id=frame_id;
            const double scale=degrees?1:std::numbers::pi/180;message.vector.x=rpy[0]*scale;message.vector.y=rpy[1]*scale;message.vector.z=rpy[2]*scale;rpy_publisher->publish(message);}
        if(imu_publisher){sensor_msgs::msg::Imu message;message.header.stamp=stamp;message.header.frame_id=frame_id;message.orientation=orientation;
            message.orientation_covariance[0]=message.orientation_covariance[4]=message.orientation_covariance[8]=.02;
            message.angular_velocity_covariance.fill(-1);message.linear_acceleration_covariance.fill(-1);imu_publisher->publish(message);}
        if(pose_publisher){geometry_msgs::msg::PoseStamped message;message.header.stamp=stamp;message.header.frame_id=frame_id;message.pose.orientation=orientation;pose_publisher->publish(message);}
        ++published;if(verbose)RCLCPP_INFO(get_logger(),"#%llu RPY(deg)=(%.2f, %.2f, %.2f)",static_cast<unsigned long long>(published),double(rpy[0]),double(rpy[1]),double(rpy[2]));
    }
    void statistics() {
        std::lock_guard lock(mutex);
        if(!error.empty()){RCLCPP_ERROR(get_logger(),"IMU reader stopped: %s",error.c_str());return;}
        if(verbose)RCLCPP_INFO(get_logger(),"IMU accepted=%llu crc=%llu invalid=%llu discarded=%llu",static_cast<unsigned long long>(counters[0]),static_cast<unsigned long long>(counters[1]),static_cast<unsigned long long>(counters[2]),static_cast<unsigned long long>(counters[3]));
    }
};
}
int main(int argc,char** argv) {
    rclcpp::init(argc,argv);int result=0;
    try{rclcpp::spin(std::make_shared<DmImuNode>());}catch(const std::exception& e){RCLCPP_FATAL(rclcpp::get_logger("dm_imu"),"%s",e.what());result=1;}
    rclcpp::shutdown();return result;
}
