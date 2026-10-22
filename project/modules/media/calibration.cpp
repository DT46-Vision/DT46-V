#include "modules/media/calibration.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <opencv2/calib3d.hpp>

namespace dt46 {
std::vector<cv::Point3f> chessboard_points(cv::Size corners,double square_metres) {
    if(corners.width<2||corners.height<2||corners.area()>10000||!std::isfinite(square_metres)||square_metres<=0)
        throw std::invalid_argument("Expected chessboard inner corners >= 2x2 and positive square size in metres");
    std::vector<cv::Point3f> points;
    for(int y=0;y<corners.height;++y)for(int x=0;x<corners.width;++x)points.emplace_back(float(x*square_metres),float(y*square_metres),0);
    return points;
}
Calibration calibrate(const std::vector<std::vector<cv::Point2f>>& observations,cv::Size image_size,cv::Size corners,double square_metres) {
    const auto board=chessboard_points(corners,square_metres);
    if(observations.size()<5||image_size.width<=0||image_size.height<=0)throw std::invalid_argument("Calibration needs at least five views and valid image dimensions");
    for(const auto& view:observations){if(view.size()!=board.size())throw std::invalid_argument("Wrong corner count");
        for(const auto& point:view)if(!std::isfinite(point.x)||!std::isfinite(point.y))throw std::invalid_argument("Nonfinite image point");}
    std::vector<std::vector<cv::Point3f>> world(observations.size(),board);std::vector<cv::Mat> rotations,translations;
    Calibration result;result.image_size=image_size;result.k=cv::Mat::eye(3,3,CV_64F);result.distortion=cv::Mat::zeros(5,1,CV_64F);
    result.rms=cv::calibrateCamera(world,observations,image_size,result.k,result.distortion,rotations,translations,0,
        {cv::TermCriteria::COUNT|cv::TermCriteria::EPS,100,1e-9});
    if(!std::isfinite(result.rms)||!cv::checkRange(result.k)||!cv::checkRange(result.distortion))throw std::runtime_error("Calibration failed to produce finite parameters");
    for(std::size_t i=0;i<observations.size();++i){std::vector<cv::Point2f> projected;cv::projectPoints(board,rotations[i],translations[i],result.k,result.distortion,projected);
        result.per_view_rms.push_back(cv::norm(observations[i],projected,cv::NORM_L2)/std::sqrt(double(board.size())));}
    return result;
}
void save_calibration(const std::filesystem::path& path,const Calibration& c,const std::string& name) {
    if(name.empty()||name.find_first_of("\r\n\"\\")!=std::string::npos)throw std::invalid_argument("Invalid camera name");
    if(path.has_parent_path())std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path);if(!file)throw std::runtime_error("Cannot open calibration output");file<<std::setprecision(17);
    file<<"image_width: "<<c.image_size.width<<"\nimage_height: "<<c.image_size.height<<"\ncamera_name: \""<<name<<"\"\ndistortion_model: plumb_bob\n";
    const auto matrix=[&](const char* key,const cv::Mat& value){file<<key<<":\n  rows: "<<value.rows<<"\n  cols: "<<value.cols<<"\n  data: [";
        for(int y=0;y<value.rows;++y)for(int x=0;x<value.cols;++x){if(y||x)file<<", ";file<<value.at<double>(y,x);}file<<"]\n";};
    matrix("camera_matrix",c.k);matrix("distortion_coefficients",c.distortion.reshape(1,1));matrix("rectification_matrix",cv::Mat::eye(3,3,CV_64F));
    cv::Mat projection=cv::Mat::zeros(3,4,CV_64F);c.k.copyTo(projection(cv::Rect(0,0,3,3)));matrix("projection_matrix",projection);
    if(!file)throw std::runtime_error("Cannot write calibration output");
}
}
