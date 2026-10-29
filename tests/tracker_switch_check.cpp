#include "rm_tracker/tracker.hpp"
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>

using namespace dt46_vision;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(int argc,char** argv){try{
    if(argc!=3)throw std::runtime_error("tracker_switch_check yaml output");
    std::ifstream input(argv[1]);const std::string source((std::istreambuf_iterator<char>(input)),{});
    cv::FileStorage yaml("%YAML:1.0\n---\n"+source,cv::FileStorage::READ|cv::FileStorage::MEMORY|cv::FileStorage::FORMAT_YAML);
    const auto params=yaml["rm_tracker"]["ros__parameters"];require(!params.empty(),"missing actual YAML");
    const std::filesystem::path out=argv[2];std::filesystem::create_directories(out);
    std::ofstream summary(out/"summary.csv");summary<<"case,matched,tracking,fire_frames,center_rmse_m,center_max_m,unobservable_body_fire_frames\n";
    bool passed=true;
    for(int mode=0;mode<4;++mode){
        Tracker tracker;tracker.cam_to_gun_pos.setZero();tracker.controller_delay_=.02;tracker.shootable_dist=10;
        params["ekf_QR_q_xyz"]>>tracker.ekf_QR_params.q_xyz;params["ekf_QR_q_yaw"]>>tracker.ekf_QR_params.q_yaw;params["ekf_QR_q_r"]>>tracker.ekf_QR_params.q_r;
        params["ekf_QR_r_xyz_factor"]>>tracker.ekf_QR_params.r_xyz_factor;params["ekf_QR_r_yaw"]>>tracker.ekf_QR_params.r_yaw;params["system_delay"]>>tracker.system_delay;
        if(!params["controller_delay"].empty())params["controller_delay"]>>tracker.controller_delay_;
        if(!params["min_hit_probability"].empty())params["min_hit_probability"]>>tracker.min_hit_probability;
        if(!params["ekf_QR_r_depth_factor"].empty())params["ekf_QR_r_depth_factor"]>>tracker.ekf_QR_params.r_depth_factor;
        params["ekf_QR_stable_dist"]>>tracker.ekf_QR_params.stable_dist;params["shootable_dist"]>>tracker.shootable_dist;
        params["max_match_distance"]>>tracker.max_match_distance;double yaw_limit=57;params["max_match_yaw_diff"]>>yaw_limit;tracker.max_match_yaw_diff=yaw_limit*DEG2RAD;
        params["radius_r_min"]>>tracker.radius_params.r_min;params["radius_r_max"]>>tracker.radius_params.r_max;
        params["tracking_thres"]>>tracker.tracking_thres;params["lost_thres"]>>tracker.lost_thres;
        params["yaw_tolerance_deg"]>>tracker.yaw_tolerance_deg;params["pitch_tolerance_deg"]>>tracker.pitch_tolerance_deg;
        params["bullet_speed"]>>tracker.bullet_speed;tracker.build_ballistic_lut();
        params["min_spinning_vel"]>>tracker.min_spinning_vel_;
        std::ofstream diagnostic(out/("case-"+std::to_string(mode)+".csv"));
        diagnostic<<"frame,time,phase,omega,cx,cy,rA,rB,matched,fire,lateral_sigma,vertical_sigma,blockers\n";
        RmTF tf;Eigen::Matrix3d rotation;rotation<<0,0,1,-1,0,0,0,-1,0;tf.set_optical_rotation(rotation);
        std::mt19937 random(1846+mode);std::normal_distribution<double> position_noise(0,.003),yaw_noise(0,mode==3?.1:.025),depth_noise(0,1);
        int matched=0,tracking=0,shots=0,partial_shots=0,count=0;double t=0,sum=0,maximum=0;
        for(int frame=0;frame<1600;++frame){
            const double dt=frame%31==0?.04:.01;t+=dt;
            const double phase=mode==0?.4:.4+4*t+(mode==2?.25*std::sin(2*t):0);
            const Eigen::Vector3d center(3+(mode==2?.15*t:0),mode==2?.2*std::sin(t):.1,-.06);
            std::vector<Armor> observations;
            for(int i=0;i<4;++i){
                const double angle=phase+i*M_PI/2,r=i%2?.32:.22;
                const Eigen::Vector3d normal(std::cos(angle),std::sin(angle),0);
                const Eigen::Vector3d truth=center-r*normal+Eigen::Vector3d(0,0,i%2?.07:0);
                if((mode==0&&i!=0)||normal.dot(truth.normalized())<.35)continue;
                Eigen::Vector3d measured=truth;for(int axis=0;axis<3;++axis)measured[axis]+=position_noise(random);
                if(mode==3)measured+=truth.normalized()*(.01*truth.squaredNorm()*depth_noise(random));
                double yaw=angle+yaw_noise(random);
                // Occasional invalid normals should be gated, not rewritten to fit the state.
                if(frame%31==0)yaw=2*std::atan2(measured.y(),measured.x())-yaw;
                if(frame%19==0)yaw+=M_PI;
                observations.emplace_back(2,measured.x(),measured.y(),measured.z(),normalize_angle(yaw));
            }
            if(frame%2==0)std::reverse(observations.begin(),observations.end());
            if(frame%43==0&&!observations.empty())observations.push_back(observations[0]);
            if(frame%97==0)observations.emplace_back(2,30,30,30,0);
            const bool missing=frame%111>=105;if(missing)observations.clear();
            const auto result=tracker.track(tf,observations,Eigen::Vector3d::Zero(),dt);const auto s=tracker.get_render_snapshot();const auto& command=std::get<0>(result);
            if(frame==0){
                BodyEkf initialized;initialized.init(s.body_state,s.body_covariance);
                const auto& a=*std::min_element(observations.begin(),observations.end(),[](const Armor& a,const Armor& b){return a.dist<b.dist;});
                const double sigma=.005+tracker.ekf_QR_params.r_xyz_factor*.01*a.dist;
                const double depth=tracker.ekf_QR_params.r_depth_factor>0?.005+tracker.ekf_QR_params.r_depth_factor*.01*a.dist*a.dist:0;
                require((initialized.plate_covariance(0)-BodyEkf::measurement_covariance(a.pos,sigma,1,depth).topLeftCorner<3,3>()).norm()<1e-10,"initial board covariance lost center/radius correlation");
            }
            require(!missing||command[2]==0,"missing observation permitted fire");
            if(frame>200){matched+=s.observed_plate>=0;tracking+=s.tracker_state==TrackerState::TRACKING;shots+=command[2]!=0;partial_shots+=command[2]!=0&&!s.body_ready;
                const double error=std::hypot(s.body_state[0]-center.x(),s.body_state[2]-center.y());sum+=error*error;maximum=std::max(maximum,error);++count;}
            if(command[2])require(s.fire_blockers.empty(),"fire and blockers disagree");
            diagnostic<<frame<<','<<t<<','<<s.body_state[6]<<','<<s.body_state[7]<<','<<s.body_state[0]<<','<<s.body_state[2]<<','<<s.body_state[8]<<','<<s.body_state[9]<<','<<s.observed_plate<<','<<command[2]<<','<<s.aim_lateral_sigma<<','<<s.aim_vertical_sigma<<',';
            for(const auto& blocker:s.fire_blockers)diagnostic<<blocker<<';';diagnostic<<'\n';
            const auto direction=(rotation*Eigen::Vector3d(std::tan(command[0]*DEG2RAD),std::tan(command[1]*DEG2RAD),1)).normalized().eval();
            rotation.col(2)=direction;rotation.col(0)=direction.cross(Eigen::Vector3d::UnitZ()).normalized();rotation.col(1)=direction.cross(rotation.col(0));tf.set_optical_rotation(rotation);
        }
        const double rmse=std::sqrt(sum/count);
        summary<<mode<<','<<matched<<','<<tracking<<','<<shots<<','<<rmse<<','<<maximum<<','<<partial_shots<<'\n';summary.flush();
        passed=passed&&matched>1150&&tracking>1150&&shots>5;
        passed=passed&&(mode==0?partial_shots>5:rmse<.06&&maximum<.15);
    }
    require(passed,"actual YAML noisy switching / useful firing / body continuity regression failed; see summary.csv");
    std::cout<<"actual YAML, single plate, rejected invalid normals, noisy switching, duplicates and missing-frame gates passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
