#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "armor_model.hpp"
#include "modules/vision/pipeline.hpp"
#ifdef DT46_MODEL_DEBUG_IMAGES
#include "modules/vision/debug_images.hpp"
#endif

namespace {
cv::Matx33d matrix(const dt46::Json& json){cv::Matx33d result;for(int i=0;i<3;++i)for(int j=0;j<3;++j)result(i,j)=json.at(i).at(j);return result;}
dt46::Json matrix(const cv::Matx33d& r){dt46::Json result=dt46::Json::array();for(int i=0;i<3;++i)result.push_back({r(i,0),r(i,1),r(i,2)});return result;}
}
int main(int argc,char** argv) {
    try {
        if(argc!=3)throw std::invalid_argument("Expected repository root and mode: crops|frames");
        const std::filesystem::path root=argv[1];const std::string mode=argv[2];
        if(mode!="crops"&&mode!="frames")throw std::invalid_argument("Unknown mode");
        std::ifstream file(root/"standalone/armor_model.json");const auto config=dt46::Json::parse(file);
        cv::setNumThreads(1);
        cv::FileStorage empty("{}",cv::FileStorage::READ|cv::FileStorage::MEMORY|cv::FileStorage::FORMAT_JSON);
        dt46::ArmorModel model((root/config.at("model").get<std::string>()).string(),empty.root());
        dt46::AimPipeline pipeline(root);
        pipeline.configure({{"detector",{{"detect_color",2}}}});
        std::string line;
        while(std::getline(std::cin,line)){
            const auto request=dt46::Json::parse(line);
            cv::Mat rgb;const auto bgr=cv::imread(request.at("image").get<std::string>());
            if(bgr.empty())throw std::runtime_error("Cannot read input image");cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);
            const auto k=matrix(request.at("K"));cv::Vec<double,5> d;for(int i=0;i<5;++i)d[i]=request.at("D").at(i);
            dt46::Json result;
            if(mode=="crops"){
                const auto o=model.infer(rgb,k,d);
                result={{"position_camera_m",o.position.val},{"rotation_camera_from_board",matrix(o.rotation)},
                    {"sticker_type",o.sticker},{"team",o.team==0?"red":"blue"},{"armor_id",o.armor_id},
                    {"robot_number",o.robot_number},{"presence_score",o.presence},{"sticker_score",o.sticker_score},{"team_score",o.team_score}};
            }else{
                if(request.value("reset",true))pipeline.reset();
                const auto rotation=request.contains("optical_rotation")?matrix(request.at("optical_rotation")):cv::Matx33d(0,0,1,-1,0,0,0,-1,0);
                const auto start=std::chrono::steady_clock::now();
                result=pipeline.process(rgb,request.value("time",0.),k,d,rotation,{},request.value("target_color",0));
                result["process_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
                double times[5];dt46_stage_times(pipeline.context(),times);result["stage_ms"]=times;
#ifdef DT46_MODEL_DEBUG_IMAGES
                if(request.contains("debug_image")){
                    auto image=dt46::tracking_image(rgb,result);cv::cvtColor(image,image,cv::COLOR_RGB2BGR);
                    if(!cv::imwrite(request.at("debug_image").get<std::string>(),image))throw std::runtime_error("Cannot save debug image");
                }
#endif
            }
            std::cout<<result.dump()<<std::endl;
        }
        return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
