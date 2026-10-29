#include "rm_detector/detector.hpp"
#include <algorithm>
#include <filesystem>
#include <iostream>

int main(int argc,char** argv){try{
    if(argc!=4)throw std::runtime_error("inspect_detector image_raw.png classifier.onnx output_directory");
    std::filesystem::create_directories(argv[3]);
    DT46_VISION::Params p{5,2.,-35,35,1.,1.,1.8,1.,3.,2.22,.7,false,.4};
    DT46_VISION::ArmorDetector detector(1,true,120,p);
    detector.set_classifier(std::make_shared<DT46_VISION::NumberClassifier>(argv[2]));
    auto input=cv::imread(argv[1]);if(input.empty())throw std::runtime_error("missing image");
    auto lights=detector.find_lights(detector.process(input));
    std::sort(lights.begin(),lights.end(),[](const auto& a,const auto& b){return a.cx<b.cx;});
    std::cout<<"lights="<<lights.size()<<" armors="<<detector.is_armor(lights).size()<<'\n';
    for(int i=0;i<int(lights.size());++i){const auto& a=lights[i];
        std::cout<<"light "<<i<<" center="<<a.cx<<','<<a.cy<<" height="<<a.height<<" color="<<a.color<<" endpoints="<<a.up<<','<<a.down<<'\n';}
    for(int i=0;i<int(lights.size());++i)for(int j=i+1;j<int(lights.size());++j){
        const auto& a=lights[i];const auto& b=lights[j];
        const double ta=std::atan2(a.down.x-a.up.x,a.down.y-a.up.y)*180/CV_PI;
        const double tb=std::atan2(b.down.x-b.up.x,b.down.y-b.up.y)*180/CV_PI;
        const double tilt=std::abs(std::remainder(ta-tb,360.));
        const double line=std::abs(std::atan2(double(b.cy-a.cy),double(b.cx-a.cx))*180/CV_PI);
        const double height_ratio=std::max(a.height,b.height)/std::min(a.height,b.height);
        const double spacing=cv::norm(cv::Point2d(a.cx,a.cy)-cv::Point2d(b.cx,b.cy))/((a.height+b.height)*.5);
        const auto result=detector.is_close(a,b);
        std::cout<<"pair "<<i<<','<<j<<" tilt="<<tilt<<" line="<<line<<" height_ratio="<<height_ratio<<" spacing_height="<<spacing<<" class="<<result.first.class_id<<" score="<<result.first.confidence<<'\n';
        if(height_ratio<1.8&&spacing>1&&spacing<3&&tilt<=35&&line<=35){
            const auto name=std::string(argv[3])+"/pair-"+std::to_string(i)+"-"+std::to_string(j);
            if(!detector.img_armor.empty())cv::imwrite(name+".png",detector.img_armor);
            if(!detector.img_armor_processed.empty())cv::imwrite(name+"-binary.png",detector.img_armor_processed);
        }
    }
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
