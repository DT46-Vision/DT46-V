#include "modules/vision/debug_images.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
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
void outlined_text(cv::Mat& image,const std::string& text,cv::Point origin,double scale,const cv::Scalar& color) {
    const int thickness=std::max(1,int(std::lround(2*scale)));
    cv::putText(image,text,origin,cv::FONT_HERSHEY_SIMPLEX,scale,cv::Scalar(0,0,0),thickness+2,cv::LINE_AA);
    cv::putText(image,text,origin,cv::FONT_HERSHEY_SIMPLEX,scale,color,thickness,cv::LINE_AA);
}
}
cv::Mat tracking_image(const cv::Mat& rgb,const Json& result,double text_size) {
    cv::Mat output=rgb.clone();
    text_size=std::min(text_size,1.2);
    outlined_text(output,"yellow: detected   magenta: raw PnP   cyan/green/orange: EKF",{12,24},.42*text_size,{255,255,255});
    int row=0;
    for(const auto& armor:result.at("observations")) {
        std::vector<cv::Point> corners;for(int i:{0,1,3,2})corners.emplace_back(int(std::nearbyint(armor.at("corners_px").at(i).at(0).get<double>())),int(std::nearbyint(armor.at("corners_px").at(i).at(1).get<double>())));
        cv::polylines(output,corners,true,cv::Scalar(255,220,0),2);
        if(armor.contains("pnp")){
            const auto& pose=armor.at("pnp");std::vector<cv::Point> pixels;
            for(int i:{0,1,3,2})if(auto p=project(pose.at("corners_camera_m").at(i),result))pixels.push_back(*p);
            if(pixels.size()==4){cv::polylines(output,pixels,true,cv::Scalar(255,0,255),1);for(const auto& p:pixels)cv::circle(output,p,3,{255,0,255},1);}
            const auto& p=armor.at("position_camera_m");const auto& n=armor.at("normal_camera");
            Json tip=Json::array();for(int i=0;i<3;++i)tip.push_back(p.at(i).get<double>()+.12*n.at(i).get<double>());
            const auto start=project(p,result),end=project(tip,result);if(start&&end)cv::arrowedLine(output,*start,*end,{255,0,255},2,cv::LINE_AA,0,.2);
            outlined_text(output,"PnP "+std::to_string(row)+": z="+number(p.at(2))+" m, reproj="+number(pose.at("reprojection_error_px"))+" px",{12,48+24*row},.45*text_size,{255,0,255});
            ++row;
        }
    }
    const auto& debug=result.at("tracker_debug");
    if(debug.contains("body_center_camera_m"))if(auto center=project(debug.at("body_center_camera_m"),result)){
        cv::drawMarker(output,*center,{0,255,255},cv::MARKER_CROSS,24,2);
        outlined_text(output,"EKF center",*center+cv::Point(12,-12),.4*text_size,{0,255,255});
        for(const auto& plate:debug.at("body_plates")){
            std::vector<cv::Point> corners;
            for(const auto& vertex:plate.at("corners_camera_m")){auto pixel=project(vertex,result);if(pixel)corners.push_back(*pixel);}
            if(corners.size()!=4)continue;
            const cv::Scalar color=plate.at("selected").get<int>()?cv::Scalar(255,160,0):plate.at("matched").get<int>()?cv::Scalar(0,255,80):cv::Scalar(0,180,220);
            cv::polylines(output,corners,true,color,2);cv::line(output,*center,(corners[0]+corners[2])/2,color,1);
            outlined_text(output,"E"+std::to_string(plate.at("index").get<int>()),corners[0],.35*text_size,color);
        }
    }
    return output;
}
cv::Mat ballistic_image(const cv::Mat& rgb,const Json& result,double text_size) {
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
    for(std::size_t row=0;row<lines.size();++row){const cv::Point point(18,int((28+26*int(row))*text_size));const auto color=row==lines.size()-1?(fire?cv::Scalar(80,255,120):cv::Scalar(255,210,90)):cv::Scalar(255,255,255);
        outlined_text(output,lines[row],point,.6*text_size,color);}
    if(debug.contains("fire_blockers")){
        int row=int(lines.size());
        for(const auto& reason:debug.at("fire_blockers"))outlined_text(output,reason.get<std::string>(),{18,int((28+26*row++)*text_size)},.5*text_size,{255,210,90});
        outlined_text(output,"arrival sigma [m]: "+number(debug.value("aim_lateral_sigma_m",0.),3)+" / "+number(debug.value("aim_vertical_sigma_m",0.),3),{18,int((28+26*row)*text_size)},.5*text_size,{255,255,255});
    }
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
