#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>

using Json=nlohmann::json;
Eigen::Vector3d vec(const Json& a){return {a.at(0).get<double>(),a.at(1).get<double>(),a.at(2).get<double>()};}
double percentile(std::vector<double> values,double q){if(values.empty())return 0;std::sort(values.begin(),values.end());return values[std::size_t(q*(values.size()-1))];}
Json statistics(const std::vector<double>& values){double sum=0;for(double x:values)sum+=x*x;return {{"count",values.size()},{"rmse",std::sqrt(sum/std::max<std::size_t>(1,values.size()))},{"p50",percentile(values,.5)},{"p95",percentile(values,.95)},{"max",percentile(values,1)}};}
int main(int argc,char** argv){try{
    if(argc!=4)throw std::runtime_error("check_aim_truth frames.jsonl truth.jsonl output");
    std::ifstream truth_file(argv[2]),frames(argv[1]);if(!truth_file||!frames)throw std::runtime_error("Missing source logs");
    std::vector<Json> truth;for(std::string line;std::getline(truth_file,line);)truth.push_back(Json::parse(line));if(truth.empty())throw std::runtime_error("Empty truth");
    std::filesystem::path out=argv[3];std::filesystem::create_directories(out);std::ofstream csv(out/"truth-comparison.csv");
    csv<<"frame,time,state,matched,association,center_error_m,omega_error_rad_s,observation_error_m,model_plate_error_m,fire\n";
    std::size_t nearest=0;std::map<std::string,std::vector<double>> metrics;
    for(std::string line;std::getline(frames,line);){const auto f=Json::parse(line);const double t=f.at("simulation_time_s");
        while(nearest+1<truth.size()&&std::abs(truth[nearest+1].at("stamp").at("time_s").get<double>()-t)<std::abs(truth[nearest].at("stamp").at("time_s").get<double>()-t))++nearest;
        const auto& tr=truth[nearest];if(tr.at("stamp").at("generation")!=f.at("reset_generation")||std::abs(tr.at("stamp").at("time_s").get<double>()-t)>.02)continue;
        Eigen::Matrix3d rotation;Eigen::Vector3d origin;const auto& optical=f.at("input").at("optical_transform");
        for(int i=0;i<3;++i){origin[i]=optical.at(i).at(3);for(int j=0;j<3;++j)rotation(i,j)=optical.at(i).at(j);}
        const auto& d=f.at("tracker_debug");const auto& state=d.at("body_ekf_state");
        const Eigen::Vector3d center=vec(tr.at("centre_world_m"))-origin;
        const double range=center.head<2>().norm();
        const double center_error=std::hypot(state.at(0).get<double>()-center.x(),state.at(2).get<double>()-center.y());
        const double omega_error=std::abs(state.at(7).get<double>()-tr.at("yaw_rate_rad_s").get<double>());
        std::vector<Eigen::Vector3d> boards;for(const auto& a:tr.at("armors"))boards.push_back(vec(a.at("position_world_m")));
        double obs_error=-1,model_error=-1;for(const auto& obs:f.at("observations")){
            if(obs.at("armor_id")!=f.at("tracked_id"))continue;
            const auto measured=(origin+rotation*vec(obs.at("position_camera_m"))).eval();double error=1e9;Eigen::Vector3d closest=boards.front();
            for(const auto& board:boards)if((board-measured).norm()<error){error=(board-measured).norm();closest=board;}
            metrics["observation_error_m"].push_back(error);obs_error=std::max(obs_error,error);
            if(error<.1){
                const Eigen::Vector3d inward=vec(tr.at("centre_world_m"))-closest;
                const double expected=std::atan2(inward.y(),inward.x());
                double raw=std::atan2(rotation(1,2),rotation(0,2))-obs.at("yaw_deg").get<double>()*3.141592653589793/180;
                if(obs.contains("normal_camera")){const Eigen::Vector3d n=rotation*vec(obs.at("normal_camera"));if(n.head<2>().norm()>1e-6)raw=std::atan2(-n.y(),-n.x());}
                const double bearing=std::atan2(measured.y()-origin.y(),measured.x()-origin.x());double yaw_error=1e9;
                for(double candidate:{raw,2*bearing-raw,raw+3.141592653589793,2*bearing-raw+3.141592653589793})yaw_error=std::min(yaw_error,std::abs(std::remainder(candidate-expected,2*3.141592653589793)));
                metrics["best_branch_observation_yaw_error_rad"].push_back(yaw_error);
            }
        }
        std::string association;for(const auto& log:d.at("logs"))if(log.at("name")=="body_association")association=log.at("value");
        if(d.at("observed_plate").get<int>()>=0&&f.at("state")=="TRACKING"){
            metrics["tracking_center_error_m"].push_back(center_error);metrics["tracking_omega_error_rad_s"].push_back(omega_error);
            metrics["tracking_relative_center_error"].push_back(center_error/std::max(.2,range));
            metrics[range<4?"near_tracking_center_error_m":"far_tracking_center_error_m"].push_back(center_error);
            if(f.at("can_fire").get<bool>()){
                metrics["fire_center_error_m"].push_back(center_error);
                metrics["fire_omega_error_rad_s"].push_back(omega_error);
            }
            for(const auto& model:d.at("body_plates"))if(model.at("matched").get<int>()){
                Eigen::Vector3d point=Eigen::Vector3d::Zero();for(const auto& vertex:model.at("corners_camera_m"))point+=vec(vertex)*.25;
                const auto world=(origin+rotation*point).eval();double error=1e9;for(const auto& board:boards)error=std::min(error,(world-board).norm());
                metrics["matched_model_plate_error_m"].push_back(error);model_error=std::max(model_error,error);
                if(f.at("can_fire").get<bool>())metrics["fire_matched_model_plate_error_m"].push_back(error);
            }
        }
        csv<<f.at("frame_number")<<','<<t<<','<<f.at("state").get<std::string>()<<','<<d.at("observed_plate")<<','<<association<<','<<center_error<<','<<omega_error<<','<<obs_error<<','<<model_error<<','<<f.at("can_fire")<<'\n';
    }
    Json summary;for(const auto& [name,values]:metrics)summary[name]=statistics(values);
    std::ofstream(out/"truth-summary.json")<<summary.dump(2);std::cout<<summary.dump()<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
