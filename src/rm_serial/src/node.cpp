#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rm_interfaces/msg/decision.hpp>
#include <rm_interfaces/msg/gimbal_control.hpp>
#include "modules/hardware/protocol.hpp"
#include "modules/hardware/serial_port.hpp"

namespace {
using namespace std::chrono_literals;
namespace hardware=dt46::hardware;
class SerialNode : public rclcpp::Node {
public:
    SerialNode():Node("serial_node") {
        port_name=declare_parameter<std::string>("port_name","/dev/ttyACM0");baud=declare_parameter<int>("baudrate",115200);
        const double read_timeout=declare_parameter<double>("timeout",1),write_seconds=declare_parameter<double>("write_timeout",1);
        if(!std::isfinite(read_timeout)||read_timeout<0||!std::isfinite(write_seconds)||write_seconds<0||write_seconds>60)throw std::invalid_argument("Invalid serial timeout");
        write_timeout=std::chrono::milliseconds(int(write_seconds*1000));
        const auto flow=declare_parameter<std::string>("flow_control","none"),parity=declare_parameter<std::string>("parity","none"),stops=declare_parameter<std::string>("stop_bits","1");
        const int rx=declare_parameter<int>("serial_receive_header",0xa5),tx=declare_parameter<int>("serial_send_header",0x5a);
        if(flow!="none"||parity!="none"||stops!="1"||rx!=0xa5||tx!=0x5a)throw std::invalid_argument("MCU protocol requires 8N1, no flow control, A5/5A headers");
        publish_rpy=declare_parameter<bool>("pub_rpy",false);
        const auto qos=rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        decision=create_publisher<rm_interfaces::msg::Decision>("/nav/decision",qos);
        if(publish_rpy)rpy=create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/rpy",qos);
        control=create_subscription<rm_interfaces::msg::GimbalControl>("tracker/gimbal_control",rclcpp::SensorDataQoS(),
            [this](rm_interfaces::msg::GimbalControl::ConstSharedPtr value){std::lock_guard lock(command_mutex);command={value->yaw,value->pitch,value->can_fire};});
        port.open(port_name,baud,write_timeout);RCLCPP_INFO(get_logger(),"MCU serial opened: %s @ %d, transmit interval 5 ms",port_name.c_str(),baud);
        worker=std::jthread([this](std::stop_token stop){run(stop);});
    }
    ~SerialNode()override{worker.request_stop();if(worker.joinable())worker.join();}
private:
    std::string port_name;int baud=115200;bool publish_rpy=false;std::chrono::milliseconds write_timeout{1000};
    hardware::SerialPort port;hardware::McuParser parser;hardware::GimbalCommand command;std::mutex command_mutex;
    rclcpp::Publisher<rm_interfaces::msg::Decision>::SharedPtr decision;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy;
    rclcpp::Subscription<rm_interfaces::msg::GimbalControl>::SharedPtr control;std::jthread worker;
    void publish(const hardware::McuFeedback& frame) {
        const auto stamp=now();rm_interfaces::msg::Decision message;message.header.stamp=stamp;message.header.frame_id="serial_receive_frame";message.color=frame.color;message.bullet_speed=frame.speed;decision->publish(message);
        if(rpy){geometry_msgs::msg::Vector3Stamped pose;pose.header.stamp=stamp;pose.header.frame_id="imu_link";pose.vector.x=frame.roll;pose.vector.y=frame.pitch;pose.vector.z=frame.yaw;rpy->publish(pose);}
    }
    bool reconnect(std::stop_token stop) {
        port.close();parser.reset();
        while(!stop.stop_requested()&&rclcpp::ok()) {
            try{port.open(port_name,baud,write_timeout);RCLCPP_INFO(get_logger(),"MCU serial reconnected");return true;}
            catch(const std::exception& e){RCLCPP_WARN(get_logger(),"MCU reconnect: %s",e.what());}
            for(int i=0;i<50&&!stop.stop_requested();++i)std::this_thread::sleep_for(20ms);
        }return false;
    }
    void run(std::stop_token stop) {
        std::array<std::uint8_t,4096> bytes{};auto next_send=std::chrono::steady_clock::now();std::uint64_t reported_crc=0;
        while(!stop.stop_requested()&&rclcpp::ok()) {
            try {
                const auto count=port.read(bytes);for(const auto& frame:parser.feed(std::span(bytes).first(count)))publish(frame);
                if(parser.crc_failures!=reported_crc){if(reported_crc<5)RCLCPP_WARN(get_logger(),"MCU CRC failures: %llu",static_cast<unsigned long long>(parser.crc_failures));reported_crc=parser.crc_failures;}
                if(std::chrono::steady_clock::now()>=next_send) {
                    hardware::GimbalCommand current;{std::lock_guard lock(command_mutex);current=command;}port.write(hardware::encode(current));next_send=std::chrono::steady_clock::now()+5ms;
                }
                std::this_thread::sleep_for(1ms);
            }catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"MCU serial: %s",e.what());if(!reconnect(stop))return;next_send=std::chrono::steady_clock::now();}
        }
    }
};
}
int main(int argc,char** argv) {
    rclcpp::init(argc,argv);int result=0;
    try{rclcpp::spin(std::make_shared<SerialNode>());}catch(const std::exception& e){RCLCPP_FATAL(rclcpp::get_logger("rm_serial"),"%s",e.what());result=1;}
    rclcpp::shutdown();return result;
}
