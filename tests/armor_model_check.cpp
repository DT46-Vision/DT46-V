#include "armor_model.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
    try {
        const cv::Mat rgb(90,150,CV_8UC3,cv::Scalar(12,34,56));
        const cv::Matx33d k(900,0,74.5,0,910,44.5,0,0,1);
        cv::Matx33d crop_k;
        const auto image=dt46::ArmorModel::crop(rgb,{10,20,100,50},k,crop_k);
        if(image.size()!=cv::Size(128,128)||image.at<cv::Vec3b>(0,0)!=cv::Vec3b(20,20,20)||
           image.at<cv::Vec3b>(32,0)!=cv::Vec3b(12,34,56))throw std::runtime_error("Crop RGB/padding changed");
        // Independent projection of an off-axis ray, including pixel centres.
        const cv::Vec3d ray(.1,-.02,1);
        const auto original=k*ray,actual=crop_k*ray;
        if(std::abs(actual[0]-((original[0]-10+.5)*1.28-.5))>1e-9||
           std::abs(actual[1]-((original[1]-20+.5)*1.28-.5+32))>1e-9)
            throw std::runtime_error("Crop calibration moved the ray");
        const float axes[]={2,0,0,1,-3,0};
        const auto r=dt46::ArmorModel::decode_rotation(axes);
        if(cv::norm(cv::Mat(r),cv::Mat(cv::Matx33d(1,0,0,0,-1,0,0,0,-1)))>1e-9||std::abs(cv::determinant(r)-1)>1e-9)
            throw std::runtime_error("Board axes or orthogonalization changed");
        for(const auto& invalid:std::vector<std::array<float,6>>{{0,0,0,0,1,0},{1,0,0,2,0,0},{NAN,0,0,0,1,0}}){
            bool rejected=false;try{dt46::ArmorModel::decode_rotation(invalid.data());}catch(const std::exception&){rejected=true;}
            if(!rejected)throw std::runtime_error("Invalid rotation accepted");
        }
        std::cout<<"RGB letterbox, pixel-centre calibration and rotation validity passed\n";
        return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
