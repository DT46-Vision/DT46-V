#include "rm_tracker/tracker.hpp"
#include "rm_tracker/frame_clock.hpp"
#include "rm_detector/pnp.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <map>

using namespace dt46_vision;
using Json=nlohmann::json;
int main(int argc,char** argv){try{
    if(argc<4||argc==8)throw std::runtime_error("check_tracker_log frames.jsonl tracker.yaml output [q_xyz [r_yaw [q_yaw [light_spacing_mm light_height_mm]]]]");
    Tracker tracker;std::ifstream yaml_input(argv[2]);if(!yaml_input)throw std::runtime_error("Cannot read tracker YAML");
    const std::string yaml_text((std::istreambuf_iterator<char>(yaml_input)),{});
    cv::FileStorage yaml("%YAML:1.0\n---\n"+yaml_text,cv::FileStorage::READ|cv::FileStorage::MEMORY|cv::FileStorage::FORMAT_YAML);const auto p=yaml["rm_tracker"]["ros__parameters"];
    p["ekf_QR_q_xyz"]>>tracker.ekf_QR_params.q_xyz;p["ekf_QR_q_yaw"]>>tracker.ekf_QR_params.q_yaw;p["ekf_QR_q_r"]>>tracker.ekf_QR_params.q_r;
    p["ekf_QR_r_xyz_factor"]>>tracker.ekf_QR_params.r_xyz_factor;p["ekf_QR_r_yaw"]>>tracker.ekf_QR_params.r_yaw;p["ekf_QR_stable_dist"]>>tracker.ekf_QR_params.stable_dist;
    if(!p["ekf_QR_r_depth_factor"].empty())p["ekf_QR_r_depth_factor"]>>tracker.ekf_QR_params.r_depth_factor;
    p["system_delay"]>>tracker.system_delay;p["shootable_dist"]>>tracker.shootable_dist;p["max_match_distance"]>>tracker.max_match_distance;
    if(!p["controller_delay"].empty())p["controller_delay"]>>tracker.controller_delay_;
    double yaw_limit=57;p["max_match_yaw_diff"]>>yaw_limit;tracker.max_match_yaw_diff=yaw_limit*DEG2RAD;
    p["radius_r_min"]>>tracker.radius_params.r_min;p["radius_r_max"]>>tracker.radius_params.r_max;p["tracking_thres"]>>tracker.tracking_thres;p["lost_thres"]>>tracker.lost_thres;
    p["yaw_tolerance_deg"]>>tracker.yaw_tolerance_deg;p["pitch_tolerance_deg"]>>tracker.pitch_tolerance_deg;
    p["min_spinning_vel"]>>tracker.min_spinning_vel_;
    if(argc>4)tracker.ekf_QR_params.q_xyz=std::stod(argv[4]);
    if(argc>5)tracker.ekf_QR_params.r_yaw=std::stod(argv[5]);
    if(argc>6)tracker.ekf_QR_params.q_yaw=std::stod(argv[6]);
    const std::filesystem::path out=argv[3];std::filesystem::create_directories(out);std::ofstream csv(out/"frames.csv");
    std::ofstream audit(out/"replayed.jsonl");
    csv<<"frame,time,state,observed,selected,matched_mask,body_ready,lateral_sigma,vertical_sigma,fire,blockers\n";
    std::ifstream input(argv[1]);if(!input)throw std::runtime_error("Cannot read source frames");
    FrameClock clock;RmTF tf;int generation=-1,frames=0,fire=0,tracking=0,matched=0,ready=0;std::map<std::string,int> blockers,association;
    DT46_VISION::PNP pnp;const bool repose=argc>8;
    for(std::string line;std::getline(input,line);){auto source=Json::parse(line);const int next=source.value("reset_generation",0);
        if(next!=generation){tracker.tracker_state=TrackerState::LOST;clock=FrameClock();generation=next;pnp=DT46_VISION::PNP();if(repose)pnp.setArmorGeometry(std::stod(argv[7]),std::stod(argv[8]));}
        const auto step=clock.advance(source.at("simulation_time_s"));if(!step.accepted)continue;if(step.reset)tracker.tracker_state=TrackerState::LOST;
        const auto& sensor=source.at("input");Eigen::Matrix3d rotation;
        if(repose){
            DT46_VISION::CameraCalibration calibration{};
            for(int i=0;i<3;++i)for(int j=0;j<3;++j)calibration.k[3*i+j]=sensor.at("K").at(i).at(j);
            for(const auto& value:sensor.at("D"))calibration.d.push_back(value);
            pnp.beginFrame();
            for(auto& a:source.at("observations")){
                std::vector<cv::Point2f> corners;for(const auto& point:a.at("corners_px"))corners.emplace_back(point.at(0).get<float>(),point.at(1).get<float>());
                DT46_VISION::Light left(corners[0],corners[1],1),right(corners[2],corners[3],1);
                DT46_VISION::Armor armor(2,left,right,{1,1.f},false,{});
                const auto [x,y,z,yaw,nx,ny,nz]=pnp.processArmorCorners(&calibration,false,cv::Mat::zeros(1,1,CV_8UC3),armor,a.at("armor_id"));
                a["position_camera_m"]={x*.001,y*.001,z*.001};a["yaw_deg"]=yaw;
                a["normal_camera"]={nx,ny,nz};
            }
        }
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)rotation(i,j)=sensor.at("optical_rotation").at(i).at(j);
        tf.set_optical_rotation(rotation);for(int i=0;i<3;++i)tracker.cam_to_gun_pos[i]=sensor.at("muzzle_offset_camera_m").at(i);
        tracker.target_color=sensor.at("target_color");const double speed=source.value("bullet_speed_m_s",25.);
        if(std::abs(speed-tracker.bullet_speed)>1e-6){tracker.bullet_speed=speed;tracker.build_ballistic_lut();}
        std::vector<Armor> observations;for(const auto& a:source.at("observations")){
            const int id=a.at("armor_id");if((tracker.target_color==0&&id<6)||(tracker.target_color==1&&id>=6))continue;
            Eigen::Vector3d camera;for(int i=0;i<3;++i)camera[i]=a.at("position_camera_m").at(i);
            const auto position=(rotation*camera).eval();double yaw=normalize_angle(std::atan2(rotation(1,2),rotation(0,2))-a.at("yaw_deg").get<double>()*DEG2RAD);
            if(a.contains("normal_camera")){const auto& n=a.at("normal_camera");yaw=tf.armor_yaw_from_normal(Eigen::Vector3d(n.at(0),n.at(1),n.at(2)),Eigen::Vector3d::Zero(),yaw);}
            observations.emplace_back(id,position.x(),position.y(),position.z(),yaw);
        }
        const auto result=tracker.track(tf,observations,Eigen::Vector3d::Zero(),step.dt);const auto s=tracker.get_render_snapshot();const bool permitted=std::get<0>(result)[2]!=0;
        ++frames;fire+=permitted;tracking+=s.tracker_state==TrackerState::TRACKING;matched+=s.observed_plate>=0;ready+=s.body_ready;
        for(const auto& log:std::get<1>(result))if(log.first=="body_association")++association[log.second];
        std::string reasons;for(const auto& reason:s.fire_blockers){++blockers[reason];reasons+=reason+";";}
        csv<<source.at("frame_number")<<','<<source.at("simulation_time_s")<<','<<int(s.tracker_state)<<','<<s.observed_plate<<','<<(s.target?s.target->index:-1)<<','<<s.body_matched_mask<<','<<s.body_ready<<','<<s.aim_lateral_sigma<<','<<s.aim_vertical_sigma<<','<<permitted<<','<<reasons<<'\n';
        Json debug={{"body_ekf_state",Json::array()},{"observed_plate",s.observed_plate},{"body_plates",Json::array()},{"logs",Json::array()}};
        for(int i=0;i<11;++i)debug["body_ekf_state"].push_back(s.body_state[i]);
        for(int i=0;i<4;++i)if(s.body_matched_mask&(1u<<i)){
            const auto& x=s.body_state;const double angle=x[6]+i*M_PI/2,r=x[8+i%2];
            const Eigen::Vector3d camera=rotation.transpose()*Eigen::Vector3d(x[0]-r*std::cos(angle),x[2]-r*std::sin(angle),x[4]+(i%2)*x[10]);
            const Json point={camera.x(),camera.y(),camera.z()};debug["body_plates"].push_back({{"matched",1},{"corners_camera_m",{point,point,point,point}}});
        }
        for(const auto& entry:std::get<1>(result))debug["logs"].push_back({{"name",entry.first},{"value",entry.second}});
        audit<<Json{{"frame_number",source.at("frame_number")},{"simulation_time_s",source.at("simulation_time_s")},{"reset_generation",generation},
            {"input",sensor},{"observations",source.at("observations")},{"tracked_id",tracker.tracked_id},{"state",s.tracker_state==TrackerState::TRACKING?"TRACKING":"OTHER"},{"tracker_debug",debug},{"can_fire",permitted}}.dump()<<'\n';
    }
    const Json summary={{"frames",frames},{"fire",fire},{"tracking",tracking},{"matched",matched},{"body_ready",ready},{"q_xyz",tracker.ekf_QR_params.q_xyz},{"q_yaw",tracker.ekf_QR_params.q_yaw},{"r_yaw",tracker.ekf_QR_params.r_yaw},{"q_r",tracker.ekf_QR_params.q_r},{"blockers",blockers},{"association",association}};
    std::ofstream(out/"summary.json")<<summary.dump(2);std::cout<<summary.dump()<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
