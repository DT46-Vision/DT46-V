#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "modules/media/calibration.hpp"
#include "modules/media/recorder.hpp"

namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void calibration(const std::filesystem::path& output) {
    const cv::Size size(1280,1024),board(8,6);const auto points=dt46::chessboard_points(board,.025);
    const cv::Matx33d k(1075,0,611,0,1082,497,0,0,1);const cv::Vec<double,5> d(-.13,.06,.001,-.002,.008);
    std::vector<std::vector<cv::Point2f>> views;
    for(int i=0;i<20;++i){const cv::Vec3d r(.08*(i%5-2),.12*(i%4-1),.04*(i%3-1)),t(-.13+.025*(i%5),-.08+.025*(i%4),.45+.025*i);
        std::vector<cv::Point2f> pixels;cv::projectPoints(points,r,t,k,d,pixels);views.push_back(pixels);}
    const auto result=dt46::calibrate(views,size,board,.025);
    require(result.rms<.001,"Synthetic reprojection RMS");require(cv::norm(result.k-cv::Mat(k),cv::NORM_INF)<.02,"Recovered camera intrinsics");
    require(cv::norm(result.distortion.reshape(1,1)-cv::Mat(d).reshape(1,1),cv::NORM_INF)<.01,"Recovered distortion");
    dt46::save_calibration(output/"camera_info.yaml",result,"test_camera");
    std::cout<<"calibration RMS="<<result.rms<<" pixels\n";
    bool rejected=false;try{dt46::calibrate({views[0]},size,board,.025);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"Insufficient calibration views accepted");
}
void video(const std::filesystem::path& output) {
    const auto path=output/"roundtrip.mp4";dt46::Recorder recorder(path,20);
    for(int i=0;i<40;++i)if(recorder.due(i*.025,0)){cv::Mat rgb(120,160,CV_8UC3,cv::Scalar(40+i*3,80,160-i*2));recorder.write(rgb);}
    require(recorder.frames()==20,"Recording sample cadence");require(recorder.due(0,1),"Reset generation starts recording immediately");recorder.close();
    cv::VideoCapture input(path.string());require(input.isOpened(),"Cannot read MP4 output");cv::Mat frame;int count=0;
    while(input.read(frame)){require(frame.size()==cv::Size(160,120),"MP4 resolution changed");const auto color=cv::mean(frame);const int tick=count*2;
        require(std::abs(color[2]-(40+tick*3))<8&&std::abs(color[0]-(160-tick*2))<8,"MP4 RGB/BGR order or frame ordering");++count;}
    require(count==20,"MP4 frame count differs from written samples");std::cout<<"MP4 round trip: "<<count<<" frames\n";
}
}
int main(int argc,char** argv) {
    try{if(argc!=2)throw std::invalid_argument("output directory required");const std::filesystem::path output=argv[1];std::filesystem::create_directories(output);calibration(output);video(output);return 0;}
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
