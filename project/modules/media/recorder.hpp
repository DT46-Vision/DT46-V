#pragma once
#include <filesystem>
#include <opencv2/videoio.hpp>

namespace dt46 {
class Recorder {
public:
    Recorder(std::filesystem::path path,double fps);
    bool due(double timestamp,std::uint64_t generation);
    void write(const cv::Mat& rgb);
    void close(){writer.release();}
    std::size_t frames()const{return count;}
private:
    std::filesystem::path path;double fps,next=0;std::uint64_t generation=UINT64_MAX;
    std::size_t count=0;cv::Size size;cv::VideoWriter writer;
};
}
