#include "rm_detector/pnp.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>

using Json=nlohmann::json;
cv::Vec3d vec(const Json& p){return {p.at(0),p.at(1),p.at(2)};}
double angle(cv::Vec3d a,cv::Vec3d b){return std::acos(std::clamp(a.dot(b)/(cv::norm(a)*cv::norm(b)),-1.,1.));}
int main(int argc,char** argv){try{
    if(argc!=4&&argc!=6)throw std::runtime_error("check_pnp frames.jsonl truth.jsonl output [light_spacing_mm light_height_mm]");
    const double spacing=argc==6?std::stod(argv[4]):135.,height=argc==6?std::stod(argv[5]):58.;
    std::ifstream input(argv[1]),truth_input(argv[2]);if(!input||!truth_input)throw std::runtime_error("Missing PnP fixture");
    std::vector<Json> truth;for(std::string line;std::getline(truth_input,line);)truth.push_back(Json::parse(line));
    if(truth.empty())throw std::runtime_error("Empty truth");
    std::filesystem::create_directories(argv[3]);std::ofstream csv(std::filesystem::path(argv[3])/"pnp.csv");
    csv<<"time,selected,candidate,reprojection_px,position_error_m,normal_error_rad,selected_position_error_m,selected_normal_error_rad\n";
    DT46_VISION::PNP pnp;std::size_t near=0;int generation=-1,count=0,wrong=0;std::map<std::string,std::vector<double>> metrics;
    const float w=float(spacing*.5),h=float(height*.5);
    const std::vector<cv::Point3f> object{{-w,-h,0},{-w,h,0},{w,-h,0},{w,h,0}};
    for(std::string line;std::getline(input,line);){const auto f=Json::parse(line);const double t=f.at("simulation_time_s");
        while(near+1<truth.size()&&std::abs(truth[near+1].at("stamp").at("time_s").get<double>()-t)<std::abs(truth[near].at("stamp").at("time_s").get<double>()-t))++near;
        const auto& tr=truth[near];if(tr.at("stamp").at("generation")!=f.at("reset_generation")||std::abs(tr.at("stamp").at("time_s").get<double>()-t)>.02)continue;
        if(generation!=f.at("reset_generation")){pnp=DT46_VISION::PNP();pnp.setArmorGeometry(spacing,height);generation=f.at("reset_generation");}
        pnp.beginFrame();DT46_VISION::CameraCalibration calibration{};cv::Matx33d rotation,k;cv::Vec3d origin;
        for(int i=0;i<3;++i){origin[i]=f.at("input").at("optical_transform").at(i).at(3);for(int j=0;j<3;++j){rotation(i,j)=f.at("input").at("optical_rotation").at(i).at(j);calibration.k[3*i+j]=k(i,j)=f.at("input").at("K").at(i).at(j);}}
        for(const auto& d:f.at("input").at("D"))calibration.d.push_back(d);
        for(const auto& a:f.at("observations")){
            std::vector<cv::Point2f> corners;for(const auto& c:a.at("corners_px"))corners.emplace_back(c.at(0).get<float>(),c.at(1).get<float>());
            DT46_VISION::Armor armor(2,DT46_VISION::Light(corners[0],corners[1],1),DT46_VISION::Light(corners[2],corners[3],1),{1,1.f},false,{});
            const auto [x,y,z,yaw,nx,ny,nz]=pnp.processArmorCorners(&calibration,false,cv::Mat(),armor,a.at("armor_id"));
            if(z<=0)continue;const cv::Vec3d measured=origin+rotation*cv::Vec3d(x,y,z)*.001;
            cv::Vec3d board;double closest=1e9;for(const auto& b:tr.at("armors")){const auto point=vec(b.at("position_world_m"));if(cv::norm(point-measured)<closest){closest=cv::norm(point-measured);board=point;}}
            if(closest>.2)continue;
            cv::Vec3d normal=board-vec(tr.at("centre_world_m"));normal[2]=0;normal*=std::cos(CV_PI/12)/cv::norm(normal);normal[2]=std::sin(CV_PI/12);
            std::vector<cv::Mat> rvecs,tvecs;cv::solvePnPGeneric(object,corners,k,calibration.d,rvecs,tvecs,false,cv::SOLVEPNP_IPPE);
            std::vector<double> errors;int selected=-1;double selected_angle=angle(rotation*cv::Vec3d(nx,ny,nz),normal);
            for(std::size_t i=0;i<rvecs.size();++i){cv::Mat r;cv::Rodrigues(rvecs[i],r);const cv::Vec3d n(r.at<double>(0,2),r.at<double>(1,2),r.at<double>(2,2));
                if(angle(n,{nx,ny,nz})<1e-5)selected=int(i);
                std::vector<cv::Point2f> projected;cv::projectPoints(object,rvecs[i],tvecs[i],k,calibration.d,projected);double sq=0;for(int j=0;j<4;++j)sq+=cv::norm(projected[j]-corners[j])*cv::norm(projected[j]-corners[j]);errors.push_back(std::sqrt(sq/4));
                const cv::Vec3d pos(tvecs[i].at<double>(0),tvecs[i].at<double>(1),tvecs[i].at<double>(2));
                csv<<t<<','<<(angle(n,{nx,ny,nz})<1e-5)<<','<<i<<','<<errors.back()<<','<<cv::norm(origin+rotation*pos*.001-board)<<','<<angle(rotation*n,normal)<<','<<closest<<','<<selected_angle<<'\n';
            }
            if(selected<0)throw std::runtime_error("Selected pose missing from IPPE candidates");
            ++count;wrong+=selected!=int(std::min_element(errors.begin(),errors.end())-errors.begin());
            metrics["position_error_m"].push_back(closest);metrics["normal_error_rad"].push_back(selected_angle);metrics["reprojection_px"].push_back(errors[selected]);
        }
    }
    Json summary={{"observations",count},{"selected_higher_reprojection",wrong},{"light_spacing_mm",spacing},{"light_height_mm",height},{"scope","Offline only; nearest truth within 20 ms, candidate nearest a board within 0.2 m; AM02 outward normal tilted 15 degrees. No truth is used online."}};
    for(auto& [name,v]:metrics){std::sort(v.begin(),v.end());double sq=0;for(double x:v)sq+=x*x;summary[name]={{"rmse",std::sqrt(sq/v.size())},{"p50",v[v.size()/2]},{"p95",v[std::size_t(.95*(v.size()-1))]}};}
    std::ofstream(std::filesystem::path(argv[3])/"summary.json")<<summary.dump(2);std::cout<<summary.dump()<<'\n';return count?0:1;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
