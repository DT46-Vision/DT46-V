#include "rm_tracker/tracker.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

using namespace dt46_vision;
void require(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
int main(int argc,char** argv){try{
    const std::filesystem::path out=argc>1?argv[1]:"tracker_model_results";std::filesystem::create_directories(out);
    std::ofstream csv(out/"frames.csv");csv<<"frame,time_s,center_error_m,fire,state,selected_plate,prediction_error_m\n";
    std::ofstream inputs(out/"inputs.csv");inputs<<std::setprecision(17)<<"frame,dt,id,x,y,z,yaw\n";
    Tracker tracker;tracker.cam_to_gun_pos.setZero();tracker.system_delay=tracker.controller_delay_=0;
    tracker.lost_thres=30;tracker.shootable_dist=10;
    RmTF tf;Eigen::Matrix3d rotation;rotation<<0,0,1,-1,0,0,0,-1,0;tf.set_optical_rotation(rotation);
    double t=0,maxerr=0,sum=0,maxlead=0;int count=0,shots=0;
    for(int frame=0;frame<900;++frame){
        const double dt=frame%30==0?.04:.01;t+=dt;
        std::vector<Armor> observations;
        const Eigen::Vector3d center(3+.1*t,.05*std::sin(t),-.06);
        for(int i=0;i<4;++i){const double yaw=.2+3*t+i*M_PI/2,r=i%2?.32:.22;
            const Eigen::Vector3d n(std::cos(yaw),std::sin(yaw),0);
            const Eigen::Vector3d p=center-r*n+Eigen::Vector3d(0,0,i%2?.07:0);
            if(n.dot(p.normalized())>.25)observations.emplace_back(8,p.x(),p.y(),p.z(),normalize_angle(yaw));}
        for(const auto& observation:observations)inputs<<frame<<','<<dt<<','<<observation.id<<','<<observation.pos.x()<<','<<observation.pos.y()<<','<<observation.pos.z()<<','<<observation.yaw<<'\n';
        const auto result=tracker.track(tf,observations,Eigen::Vector3d::Zero(),dt);
        const auto snapshot=tracker.get_render_snapshot();double error=-1,lead=-1;
        if(frame>100&&snapshot.tracker_state!=TrackerState::LOST){error=std::hypot(snapshot.target_state[0]-center.x(),snapshot.target_state[2]-center.y());
            sum+=error*error;maxerr=std::max(maxerr,error);++count;}
        const auto& cmd=std::get<0>(result);if(cmd[2])++shots;
        if(snapshot.target){
            // Predicted target must remain on a physical board, rather than the cylinder front.
            const double radius=snapshot.target->index%2?.32:.22;
            const double distance=std::hypot(snapshot.target->pos.x()-snapshot.target_state[0],snapshot.target->pos.y()-snapshot.target_state[2]);
            lead=std::abs(distance-radius);if(frame>100)maxlead=std::max(maxlead,lead);
        }
        csv<<frame<<','<<t<<','<<error<<','<<cmd[2]<<','<<int(snapshot.tracker_state)<<','<<(snapshot.target?snapshot.target->index:-1)<<','<<lead<<'\n';
        // Ideal gimbal follows the published command, using only its own previous attitude.
        const Eigen::Vector3d direction=(rotation*Eigen::Vector3d(std::tan(cmd[0]*DEG2RAD),std::tan(cmd[1]*DEG2RAD),1)).normalized();
        rotation.col(2)=direction;rotation.col(0)=direction.cross(Eigen::Vector3d::UnitZ()).normalized();rotation.col(1)=direction.cross(rotation.col(0));tf.set_optical_rotation(rotation);
    }
    const double rmse=std::sqrt(sum/std::max(count,1));
    std::ofstream summary(out/"summary.txt");summary<<"frames=900\ncenter_rmse_m="<<rmse<<"\ncenter_max_m="<<maxerr<<"\nfire_frames="<<shots<<"\n";
    if(argc>2&&std::string(argv[2])=="--reference")return 0;
    require(count>700&&rmse<.03&&maxerr<.06,"actual tracker center regression failed");
    require(shots>20,"actual tracker never fired");
    auto no_frame=tracker.track(tf,{},Eigen::Vector3d::Zero(),0);require(std::get<0>(no_frame)[2]==0&&!tracker.target_cam_cache_,"zero dt reused aim");
    auto gap=tracker.track(tf,{},Eigen::Vector3d::Zero(),.8);require(std::get<0>(gap)[2]==0&&tracker.tracker_state==TrackerState::LOST,"gap did not invalidate tracking");
    const auto [yaw,pitch,valid]=tracker.solve_ballistic(tf,Armor(8,20,0,0,0),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    require(!valid,"outside LUT accepted geometric fallback");
    const auto stale=tracker.get_render_snapshot();require(stale.observed_plate<0&&stale.body_matched_mask==0,"invalid frame retained a measurement match");
    Tracker gate;gate.shootable_dist=10;
    require(std::get<2>(gate.can_fire(Armor(8,3,0,0,.6),{0,0,true})),
            "Visible oblique board was rejected despite aligned muzzle");
    require(!std::get<2>(gate.can_fire(Armor(8,3,0,0,M_PI),{0,0,true})),
            "Back-facing board allowed fire");
    require(!std::get<2>(gate.can_fire(Armor(8,8,0,0,0),{0,.5,true})),"far pitch gate exceeds physical board height");
    gate.yaw_tolerance_deg=.01;
    require(!std::get<2>(gate.can_fire(Armor(8,2,0,0,0),{.1,0,true})),"configured yaw limit ignored");
    require(gate.yaw_tolerance_deg==.01,"fire HUD mutated configured tolerance");
    Tracker red,blue;
    red.track(tf,{Armor(8,3,0,0,.2)},Eigen::Vector3d::Zero(),.01);
    blue.track(tf,{Armor(2,3,0,0,.2)},Eigen::Vector3d::Zero(),.01);
    require((red.get_render_snapshot().body_state-blue.get_render_snapshot().body_state).norm()==0,
            "Vehicle ID changes geometry prior");
    std::cout<<"actual tracker center RMSE "<<rmse<<", max "<<maxerr<<", fire frames "<<shots<<'\n';return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
