#include "modules/vision/debug_images.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <iomanip>
#include <sstream>

namespace dt46 {
namespace {
std::optional<cv::Point> project(const cv::Vec3d& point,const cv::Matx33d& k,const cv::Vec<double,5>& d) {
    if(point[2]<=0||!std::isfinite(cv::norm(point)))return {};std::vector<cv::Point2d> pixels;
    cv::projectPoints(std::vector<cv::Point3d>{{point[0],point[1],point[2]}},cv::Vec3d(),cv::Vec3d(),k,d,pixels);
    const auto& pixel=pixels[0];if(!std::isfinite(pixel.x+pixel.y)||std::max(std::abs(pixel.x),std::abs(pixel.y))>=1e7)return {};
    return cv::Point(int(std::nearbyint(pixel.x)),int(std::nearbyint(pixel.y)));
}
std::optional<cv::Point> project(const Json& point,const Json& result) {
    cv::Matx33d k;cv::Vec<double,5> d;for(int i=0;i<3;++i)for(int j=0;j<3;++j)k(i,j)=result.at("input").at("K").at(i).at(j);
    for(int i=0;i<5;++i)d[i]=result.at("input").at("D").at(i);
    return project(cv::Vec3d(point.at(0),point.at(1),point.at(2)),k,d);
}
std::string number(double value,int precision=2){std::ostringstream output;output<<std::fixed<<std::setprecision(precision)<<value;return output.str();}
}
cv::Mat tracking_image(const cv::Mat& rgb,const Json& result) {
    cv::Mat output=rgb.clone();
    for(const auto& armor:result.at("observations")) {
        std::vector<cv::Point> corners;for(int i:{0,1,3,2})corners.emplace_back(int(std::nearbyint(armor.at("corners_px").at(i).at(0).get<double>())),int(std::nearbyint(armor.at("corners_px").at(i).at(1).get<double>())));
        cv::polylines(output,corners,true,cv::Scalar(255,220,0),2);
    }
    for(const auto& armor:result.at("tracker_debug").at("yaw_armors"))if(auto point=project(armor.at("position_camera_m"),result))
        cv::putText(output,"yaw "+number(armor.at("yaw_rad"),1)+" rad",*point,cv::FONT_HERSHEY_SIMPLEX,.55,cv::Scalar(255,0,255),2);
    return output;
}
cv::Mat ballistic_image(const cv::Mat& rgb,const Json& result) {
    cv::Mat output=rgb.clone();cv::drawMarker(output,{output.cols/2,output.rows/2},{255,0,255},cv::MARKER_CROSS,40,2);const auto& debug=result.at("tracker_debug");
    for(const char* key:{"target_camera_m","muzzle_target_camera_m"})if(debug.contains(key))if(auto point=project(debug.at(key),result)) {
        if(std::string(key)=="target_camera_m")cv::circle(output,*point,18,{255,0,0},2);else cv::drawMarker(output,*point,{0,255,0},cv::MARKER_CROSS,25,2);
    }
    const bool fire=result.at("can_fire");
    const std::vector<std::string> lines={result.at("state"),debug.at("spin").get<int>()?"SPIN":"NORMAL",
        "Armor w: "+number(debug.value("ekf_yaw_velocity_rad_s",0.))+" rad/s","Dist: "+number(debug.value("target_distance_m",0.))+" m",
        "Diff: "+number(debug.value("target_angle_diff_deg",0.),1)+" deg","Pitch: "+number(debug.at("gimbal_control").at(1))+" deg",
        "Yaw: "+number(debug.at("gimbal_control").at(0))+" deg","P/Y Tol: "+number(debug.at("pitch_tolerance_deg"))+" / "+number(debug.at("yaw_tolerance_deg"))+" deg",
        "Bullet Speed: "+number(debug.at("bullet_speed_m_s"))+" m/s",fire?"FIRE ENABLE":"HOLD FIRE"};
    for(std::size_t row=0;row<lines.size();++row){const cv::Point point(18,28+26*int(row));const auto color=row==lines.size()-1&&fire?cv::Scalar(0,255,0):cv::Scalar(255,255,255);
        cv::putText(output,lines[row],point,cv::FONT_HERSHEY_SIMPLEX,.6,{0,0,0},3);cv::putText(output,lines[row],point,cv::FONT_HERSHEY_SIMPLEX,.6,color,1);}
    return output;
}
cv::Mat observer_image(const cv::Mat& rgb,const Json& result,const cv::Matx44d& optical,const cv::Matx33d& k,const cv::Vec<double,5>& d) {
    cv::Mat output=rgb.clone();const auto& debug=result.at("tracker_debug");if(!debug.contains("target_camera_m"))return output;
    cv::Vec4d target(debug.at("target_camera_m").at(0),debug.at("target_camera_m").at(1),debug.at("target_camera_m").at(2),1);cv::Matx44d source;
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)source(i,j)=result.at("input").at("optical_transform").at(i).at(j);
    const auto observer=optical.inv()*source*target;
    if(auto point=project(cv::Vec3d(observer[0],observer[1],observer[2]),k,d)){cv::circle(output,*point,12,{255,0,0},2);cv::putText(output,"tracker prediction",*point+cv::Point(14,0),cv::FONT_HERSHEY_SIMPLEX,.55,{255,0,0},2);}return output;
}
}
