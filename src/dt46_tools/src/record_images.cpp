#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

namespace {
using Clock=std::chrono::steady_clock;
struct Options {std::string topic;std::filesystem::path out,images;double fps=20,idle=8,interval=1;std::size_t count=0;};
Options parse(const std::vector<std::string>& args) {
    Options o;
    for(std::size_t i=1;i<args.size();++i){const auto& arg=args[i];const auto value=[&]{if(++i>=args.size())throw std::invalid_argument("Missing value: "+arg);return args[i];};
        if(arg=="--topic")o.topic=value();else if(arg=="--out")o.out=value();else if(arg=="--images")o.images=value();else if(arg=="--fps")o.fps=std::stod(value());
        else if(arg=="--idle")o.idle=std::stod(value());else if(arg=="--interval")o.interval=std::stod(value());else if(arg=="--count")o.count=std::stoull(value());
        else throw std::invalid_argument("Unknown argument: "+arg);}
    if(o.topic.empty()||o.out.empty()==o.images.empty())throw std::invalid_argument("Specify --topic and exactly one of --out VIDEO.mp4 or --images DIRECTORY");
    if(!std::isfinite(o.fps)||o.fps<=0||!std::isfinite(o.idle)||o.idle<=0||!std::isfinite(o.interval)||o.interval<=0)throw std::invalid_argument("Rates and timeouts must be positive and finite");
    return o;
}
class Recorder:public rclcpp::Node {
public:
    explicit Recorder(Options options):Node("tracker_mp4_recorder"),o(std::move(options)) {
        if(!o.images.empty())std::filesystem::create_directories(o.images);else if(o.out.has_parent_path())std::filesystem::create_directories(o.out.parent_path());
        subscription=create_subscription<sensor_msgs::msg::Image>(o.topic,rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Image::ConstSharedPtr message){
            try{receive(message);}catch(const std::exception& e){failure=e.what();done=true;}});
        timer=create_wall_timer(std::chrono::milliseconds(100),[this]{if(frames&&std::chrono::duration<double>(Clock::now()-last).count()>o.idle)done=true;});
    }
    bool done=false;std::string failure;std::size_t frames=0;
    void close(){writer.release();RCLCPP_INFO(get_logger(),"Saved %zu images",frames);}
private:
    Options o;cv::VideoWriter writer;cv::Size size;Clock::time_point last=Clock::now(),next{};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;rclcpp::TimerBase::SharedPtr timer;
    void receive(const sensor_msgs::msg::Image::ConstSharedPtr& message) {
        if(done)return;last=Clock::now();const auto bgr=cv_bridge::toCvShare(message,"bgr8")->image;
        if(!o.images.empty()) {
            if(last<next)return;next=last+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(o.interval));
            if(!cv::imwrite((o.images/("frame_"+std::to_string(frames)+".png")).string(),bgr))throw std::runtime_error("Cannot write calibration image");
        }else {
            if(!writer.isOpened()){size=bgr.size();if(!writer.open(o.out.string(),cv::VideoWriter::fourcc('m','p','4','v'),o.fps,size))throw std::runtime_error("Cannot open MP4 writer");}
            if(bgr.size()!=size)throw std::runtime_error("Image resolution changed during recording");writer.write(bgr);
        }
        ++frames;if(o.count&&frames>=o.count)done=true;
    }
};
}
int main(int argc,char** argv) {
    rclcpp::init(argc,argv);int status=1;
    try {
        auto node=std::make_shared<Recorder>(parse(rclcpp::remove_ros_arguments(argc,argv)));
        rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
        while(rclcpp::ok()&&!node->done)executor.spin_once(std::chrono::milliseconds(100));node->close();
        if(!node->failure.empty())throw std::runtime_error(node->failure);status=node->frames?0:1;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';}
    rclcpp::shutdown();return status;
}
