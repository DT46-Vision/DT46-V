#include "modules/media/recorder.hpp"
#include <cmath>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace dt46 {
Recorder::Recorder(std::filesystem::path output,double rate):path(std::move(output)),fps(rate) {
    if(path.empty()||!std::isfinite(fps)||fps<=0)throw std::invalid_argument("Recording requires a path and positive finite FPS");
}
bool Recorder::due(double timestamp,std::uint64_t current_generation) {
    if(!std::isfinite(timestamp)||timestamp<0)throw std::invalid_argument("Invalid recording timestamp");
    if(current_generation!=generation){generation=current_generation;next=timestamp;}
    if(timestamp+1e-10<next)return false;
    // One image per actual sample. A slow source is never padded with stale duplicates.
    next+=1/fps;if(next<timestamp-1/fps)next=timestamp+1/fps;return true;
}
void Recorder::write(const cv::Mat& rgb) {
    if(rgb.empty()||rgb.type()!=CV_8UC3)throw std::invalid_argument("Recording expects RGB8");
    if(!writer.isOpened()) {
        if(path.has_parent_path())std::filesystem::create_directories(path.parent_path());size=rgb.size();
        if(!writer.open(path.string(),cv::VideoWriter::fourcc('m','p','4','v'),fps,size))throw std::runtime_error("Cannot open MP4 writer: "+path.string());
    }
    if(rgb.size()!=size)throw std::invalid_argument("Recording resolution changed");
    cv::Mat bgr;cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);writer.write(bgr);++count;
}
}
