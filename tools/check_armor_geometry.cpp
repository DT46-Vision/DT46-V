#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>
using Json=nlohmann::json;
Eigen::Vector3d vector3(const Json& v){return {v.at(0).get<double>(),v.at(1).get<double>(),v.at(2).get<double>()};}
double quantile(std::vector<double> v,double p){std::sort(v.begin(),v.end());return v.at(std::size_t(p*(v.size()-1)));}
int main(int argc,char** argv){try{
    if(argc!=4&&argc!=5)throw std::runtime_error("check_armor_geometry stationary_frames.jsonl stationary_truth.jsonl output.json [--all]");
    const bool all=argc==5&&std::string(argv[4])=="--all";
    if(argc==5&&!all)throw std::runtime_error("unknown geometry audit option");
    std::ifstream frames(argv[1]),source(argv[2]);if(!frames||!source)throw std::runtime_error("missing fixture");
    std::vector<Json> truth;for(std::string line;std::getline(source,line);)truth.push_back(Json::parse(line));
    if(truth.empty())throw std::runtime_error("empty stationary truth");
    std::vector<double> width,height;std::size_t nearest=0;std::optional<double> first;
    Json samples=Json::array();
    for(std::string line;std::getline(frames,line);){const auto f=Json::parse(line);const double t=f.at("simulation_time_s");if(!first&&!f.at("observations").empty())first=t;if(!all&&first&&t>*first+.6)break;
        while(nearest+1<truth.size()&&std::abs(truth[nearest+1].at("stamp").at("time_s").get<double>()-t)<std::abs(truth[nearest].at("stamp").at("time_s").get<double>()-t))++nearest;
        const auto& tr=truth.at(nearest);
        if(tr.at("stamp").at("generation")!=f.at("reset_generation")||std::abs(tr.at("stamp").at("time_s").get<double>()-t)>.02)continue;
        if(vector3(tr.at("velocity_world_m_s")).norm()>1e-6||std::abs(tr.at("yaw_rate_rad_s").get<double>())>1e-6)throw std::runtime_error("fixture must be stationary");
        Eigen::Matrix3d rotation;Eigen::Vector3d origin;const auto& sensor=f.at("input");
        cv::Mat k(3,3,CV_64F),d(1,5,CV_64F);
        for(int i=0;i<3;++i){origin[i]=sensor.at("optical_transform").at(i).at(3);for(int j=0;j<3;++j){rotation(i,j)=sensor.at("optical_rotation").at(i).at(j);k.at<double>(i,j)=sensor.at("K").at(i).at(j);}}
        for(int i=0;i<5;++i)d.at<double>(i)=sensor.at("D").at(i);
        for(const auto& a:f.at("observations")){
            const Eigen::Vector3d measured=origin+rotation*vector3(a.at("position_camera_m"));
            Eigen::Vector3d board;double error=1e9;for(const auto& b:tr.at("armors")){const auto point=vector3(b.at("position_world_m"));if((point-measured).norm()<error){error=(point-measured).norm();board=point;}}
            if(error>.1)continue;
            Eigen::Vector3d normal=board-vector3(tr.at("centre_world_m"));normal.z()=0;normal.normalize();
            normal*=std::cos(15*CV_PI/180);normal.z()=std::sin(15*CV_PI/180); // Existing AM02 fixture inclination.
            std::vector<cv::Point2d> pixels,rays;for(const auto& p:a.at("corners_px"))pixels.emplace_back(p.at(0).get<double>(),p.at(1).get<double>());
            cv::undistortPoints(pixels,rays,k,d);
            std::vector<Eigen::Vector3d> points;for(const auto& r:rays){const Eigen::Vector3d ray=rotation*Eigen::Vector3d(r.x,r.y,1);points.push_back(origin+ray*(normal.dot(board-origin)/normal.dot(ray)));}
            width.push_back(.5*((points[0]-points[2]).norm()+(points[1]-points[3]).norm())*1000);
            height.push_back(.5*((points[0]-points[1]).norm()+(points[2]-points[3]).norm())*1000);
            samples.push_back({{"time_s",t},{"distance_m",(board-origin).norm()},{"width_mm",width.back()},{"height_mm",height.back()},
                {"facing_cosine",normal.dot((origin-board).normalized())}});
        }
    }
    if(width.size()<8)throw std::runtime_error("insufficient stationary fixture observations");
    Json result={{"count",width.size()},{"width_mm",{{"p05",quantile(width,.05)},{"median",quantile(width,.5)},{"p95",quantile(width,.95)}}},{"height_mm",{{"p05",quantile(height,.05)},{"median",quantile(height,.5)},{"p95",quantile(height,.95)}}},{"scope","Offline rendered stationary fixture, first 0.6 s with observations; known plane inclination 15 degrees. Not a hardware calibration."}};
    if(all){result["samples"]=samples;result["scope"]="Offline rendered stationary target, all observations; known plane inclination 15 degrees. Not a hardware calibration.";}
    std::ofstream(argv[3])<<result.dump(2);result.erase("samples");std::cout<<result.dump()<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
