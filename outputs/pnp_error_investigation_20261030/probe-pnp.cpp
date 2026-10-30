#include "rm_detector/pnp.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <random>
#include <algorithm>

using Json=nlohmann::json;
int main(int argc,char**argv){
    if(argc!=3)return 2;
    std::ifstream input(argv[1]); std::ofstream output(argv[2]);
    std::mt19937 rng(20261030); std::normal_distribution<float> noise(0,.25f);
    for(std::string line;std::getline(input,line);){
        const auto probe=Json::parse(line);const auto& sensor=probe.at("input");
        DT46_VISION::CameraCalibration calibration{};cv::Matx33d rotation;
        for(int i=0;i<3;++i)for(int j=0;j<3;++j){rotation(i,j)=sensor.at("optical_rotation").at(i).at(j);calibration.k[3*i+j]=sensor.at("K").at(i).at(j);}
        for(const auto& v:sensor.at("D"))calibration.d.push_back(v);
        const cv::Vec3d truth(probe.at("position_camera").at(0),probe.at("position_camera").at(1),probe.at("position_camera").at(2));
        const cv::Vec3d true_normal(probe.at("normal_camera").at(0),probe.at("normal_camera").at(1),probe.at("normal_camera").at(2));
        Json result={{"run",probe.at("run")},{"side",probe.at("side")}};
        std::vector<std::string> modes={"ideal","measured","noise_0.25px"};
        if(probe.contains("visible_mesh")){modes.push_back("visible_mesh");modes.push_back("uncovered_mesh");}
        for(const auto& mode:modes){
            const int trials=mode=="noise_0.25px"?1000:1;
            Json measurements=Json::array();int rejected=0;
            for(int trial=0;trial<trials;++trial){
                std::vector<cv::Point2f> corners;
                for(const auto& p:probe.at(mode=="measured"?"corners":mode=="noise_0.25px"?"ideal":mode)){
                    corners.emplace_back(p.at(0).get<float>(),p.at(1).get<float>());
                    if(trials>1){corners.back().x+=noise(rng);corners.back().y+=noise(rng);}
                }
                DT46_VISION::Armor armor(2,DT46_VISION::Light(corners[0],corners[1],1),DT46_VISION::Light(corners[2],corners[3],1),{1,1.f},false,{});
                DT46_VISION::PNP pnp;pnp.setArmorGeometry(135,58);pnp.setUprightConstraint(rotation,15);
                const auto[x,y,z,yaw,nx,ny,nz]=pnp.processArmorCorners(&calibration,false,cv::Mat(),armor,2);
                if(z<=0){++rejected;continue;}
                const cv::Vec3d error=cv::Vec3d(x,y,z)*.001-truth;
                measurements.push_back({{"depth_error_mm",error.dot(truth)/cv::norm(truth)*1000},{"position_error_mm",cv::norm(error)*1000},{"normal_error_deg",std::acos(std::clamp(cv::Vec3d(nx,ny,nz).dot(true_normal),-1.,1.))*180/CV_PI}});
            }
            result[mode]={{"trials",trials},{"rejected",rejected},{"measurements",measurements}};
        }
        output<<result.dump()<<'\n';
    }
}
