#include "modules/vision/relative_aim.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace dt46::relative;
constexpr double pi=std::numbers::pi;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Scenario {
    std::string name;double spin,ego,acceleration,imu_lag;
    bool compensate=true,single=false;
    unsigned seed=46029;double angular_noise=0;
    bool faults=false;
    double imu_compensation=-1;
};
struct Config {
    DelayConfig sensor;
    double execution=0,uncertainty=0,correct_lag=0,wrong_lag=0;
};
Config read_config(const std::filesystem::path& path){
    std::ifstream file(path);require(bool(file),"cannot open delay config");Config c;std::string line;
    while(std::getline(file,line)){
        if(line.empty()||line[0]=='#')continue;
        std::istringstream row(line);std::string key;double value;
        require(bool(row>>key>>value)&&std::isfinite(value)&&value>=0,"invalid delay config");
        if(key=="image_stamp_delay_s")c.sensor.image_stamp_delay_s=value;
        else if(key=="imu_publish_delay_s")c.sensor.imu_publish_delay_s=value;
        else if(key=="execution_delay_s")c.execution=value;
        else if(key=="stamp_error_bound_s")c.uncertainty=value;
        else if(key=="lag_case_correct_imu_delay_s")c.correct_lag=value;
        else if(key=="lag_case_wrong_imu_delay_s")c.wrong_lag=value;
        else throw std::runtime_error("unknown delay key: "+key);
    }return c;
}
// This generator owns truth. The controller receives only Frame, Timing and speed.
Vec3 center(double t,const Scenario& s){return {4+(.25-s.ego)*t+.5*s.acceleration*t*t,.12+.08*t,.45};}
Vec3 truth_plate(double t,int i,const Scenario& s){
    const double yaw=.35+s.spin*t+i*pi/2,r=i%2?.32:.22;
    return center(t,s)+Vec3(r*std::cos(yaw),r*std::sin(yaw),i%2?.07:0);
}
Eigen::Quaterniond attitude(double t){
    return Eigen::Quaterniond(Eigen::AngleAxisd(.4*std::sin(2*t),Vec3::UnitZ())*
        Eigen::AngleAxisd(.08*std::sin(t),Vec3::UnitY())*Eigen::AngleAxisd(.04*std::cos(t),Vec3::UnitX()));
}
struct Metrics {int fits=0,shots=0,hits=0,rejected=0;double sum2=0,max_error=0,max_miss=0;};
Metrics run(const Scenario& s,std::ofstream& trace,std::ofstream& summary,const Config& config){
    auto sensor=config.sensor;if(s.imu_compensation>=0)sensor.imu_publish_delay_s=s.imu_compensation;
    Tracker tracker(pi/2,Eigen::Matrix3d::Identity(),sensor);
    std::mt19937 rng(s.seed);std::normal_distribution<double> noise(0,.001);
    Metrics m;double naive_sum2=0;
    for(int frame=0;frame<600;++frame){
        const double t=frame*.01;
        const auto q=attitude(t);
        const Eigen::Quaterniond qi=attitude(t-s.imu_lag)*Eigen::Quaterniond(
            Eigen::AngleAxisd(s.angular_noise*noise(rng)/.001,Vec3::UnitZ()));
        Frame input{t+.006,t+.008,t+.012,{t+.004,s.compensate?qi:Eigen::Quaterniond::Identity()}, {}};
        for(int i=0;i<4;++i){
            const double yaw=.35+s.spin*t+i*pi/2;
            const Vec3 n(std::cos(yaw),std::sin(yaw),0),p=truth_plate(t,i,s);
            if(n.dot(-p.normalized())<.15)continue;
            Vec3 measurement=q.conjugate()*p;
            for(int j=0;j<3;++j)measurement[j]+=noise(rng);
            const Vec3 measured_normal=Eigen::AngleAxisd(s.angular_noise*noise(rng)/.001,Vec3::UnitZ())*n;
            input.plates.push_back({measurement,q.conjugate()*measured_normal});
            if(s.single)break;
        }
        if(s.faults){
            if(frame>=200&&frame<225)input.plates.clear();
            if(frame==350&&!input.plates.empty())input.plates.front().camera_position_m.x()+=1;
            if(frame%2)std::reverse(input.plates.begin(),input.plates.end());
        }
        tracker.update(input);
        double error=-1,hit_error=-1;Shot shot;
        if(tracker.body()){
            const auto& b=*tracker.body();error=(b.center-center(t,s)).head<2>().norm();
            ++m.fits;m.sum2+=error*error;m.max_error=std::max(m.max_error,error);
            Timing timing{t+.014,config.uncertainty,config.execution};
            shot=aim(tracker,timing,25,t,Vec3::UnitX());
            if(shot.valid){
                // Ideal gimbal instantly reaches the command; no response claim is made.
                const Vec3 bore(std::cos(shot.pitch_rad)*std::cos(shot.yaw_rad),
                    std::cos(shot.pitch_rad)*std::sin(shot.yaw_rad),std::sin(shot.pitch_rad));
                shot=aim(tracker,timing,25,t,bore);
                if(shot.can_fire){
                    ++m.shots;hit_error=100;
                    const double impact=t+.014+config.execution+shot.flight_s;
                    for(int i=0;i<4;++i)hit_error=std::min(hit_error,(truth_plate(impact,i,s)-shot.intercept).norm());
                    if(hit_error<.025)++m.hits;
                    m.max_miss=std::max(m.max_miss,hit_error);
                }
            }
            require(!aim(tracker,{t+.014,std::nullopt,.008},25,t,Vec3::UnitX()).can_fire,"unknown timing fired");
            require(!aim(tracker,timing,25,t-1,Vec3::UnitX()).can_fire,"stale speed fired");
            require(!aim(tracker,{t+.014,1.,.008},25,t,Vec3::UnitX()).can_fire,"large timing bound fired");
            require(!aim(tracker,timing,25,t,-Vec3::UnitX()).can_fire,"backward bore fired");
            const Vec3 opposite=tracker.plate(2,0),p=tracker.plate(0,0);
            require((opposite.head<2>()+p.head<2>()-2*b.center.head<2>()).norm()<1e-9,"opposite symmetry");
            require(std::abs(opposite.z()-p.z())<1e-9,"opposite height");
        }else ++m.rejected;
        // Same-input geometry shortcut, explicitly not the historical Glenn tracker.
        double naive=-1;
        if(!input.plates.empty()){
            const auto& p=input.plates.front();
            const Vec3 estimate=input.visual_imu.orientation*(p.camera_position_m-.25*p.camera_normal);
            naive=(estimate-center(t,s)).head<2>().norm();naive_sum2+=naive*naive;
        }
        trace<<s.name<<','<<frame<<','<<t<<','<<tracker.reason()<<','<<error<<','<<naive<<','<<shot.plate<<','<<shot.can_fire<<','<<hit_error<<'\n';
    }
    summary<<s.name<<','<<m.fits<<','<<m.rejected<<','<<(m.fits?std::sqrt(m.sum2/m.fits):-1)<<','<<m.max_error<<','
        <<std::sqrt(naive_sum2/600)<<','<<m.shots<<','<<m.hits<<','<<m.max_miss<<','
        <<sensor.image_stamp_delay_s<<','<<sensor.imu_publish_delay_s<<','<<config.execution<<'\n';
    return m;
}
int main(int argc,char** argv){try{
    const std::filesystem::path out=argc>1?argv[1]:"relative_aim_results";std::filesystem::create_directories(out);
    require(argc>2,"usage: dt46_relative_aim_check output_directory delay_config");
    const auto config=read_config(argv[2]);
    std::filesystem::copy_file(argv[2],out/"delays_used.cfg",std::filesystem::copy_options::overwrite_existing);
    std::ofstream trace(out/"frames.csv"),summary(out/"summary.csv");
    trace<<"scenario,frame,exposure_truth_s,reason,center_xy_error_m,same_input_nominal_radius_error_m,selected_plate,fire,impact_error_m\n";
    summary<<"scenario,fits,rejected,center_rmse_m,center_max_m,nominal_radius_rmse_m,shots,hits_25mm,max_impact_error_m,image_delay_s,imu_delay_s,execution_delay_s\n";
    const auto good=run({"spin",3,0,0,0},trace,summary,config);
    const auto ego=run({"ego_spin",3,.6,0,0},trace,summary,config);
    const auto fast=run({"fast_spin",8,.6,0,0},trace,summary,config);
    const auto static_dual=run({"static_dual",0,0,0,0},trace,summary,config);
    const auto static_case=run({"static_single_unobservable",0,0,0,0,true,true},trace,summary,config);
    const auto uncompensated=run({"no_rotation_compensation",3,.6,0,0,false},trace,summary,config);
    const auto lag=run({"hidden_imu_lag_80ms",3,.6,0,.08},trace,summary,config);
    const auto lag_zero=run({"imu_lag_zero_compensation",3,.6,0,.08,true,false,46029,0,false,0},trace,summary,config);
    const auto lag_correct=run({"imu_lag_configured_compensation",3,.6,0,.08,true,false,46029,0,false,config.correct_lag},trace,summary,config);
    const auto lag_wrong=run({"imu_lag_wrong_compensation",3,.6,0,.08,true,false,46029,0,false,config.wrong_lag},trace,summary,config);
    const auto all_zero=run({"all_compensation_zero",3,.6,0,.08},trace,summary,Config{});
    const auto accel=run({"relative_acceleration",3,.6,3,0},trace,summary,config);
    const auto holdout=run({"holdout_noisy_pose",-4,-.3,0,0,true,false,8127,.002},trace,summary,config);
    const auto mild=run({"holdout_mild_pose_noise",-2,-.3,0,0,true,false,19031,.0005},trace,summary,config);
    const auto faults=run({"dropout_outlier_permutation",3,.6,0,0,true,false,9031,0,true},trace,summary,config);
    require(good.fits>400 && good.max_error<.025,"spin center failed");
    require(ego.fits>400 && ego.max_error<.025,"ego relative center failed");
    require(good.shots>50 && good.hits>.95*good.shots,"no useful firing in ideal case");
    require(static_case.fits==0,"unobservable geometry claimed fit");
    require(static_dual.fits>400,"static dual geometry not recovered");
    require(uncompensated.max_error>ego.max_error*3,"rotation counterexample absent");
    require(lag.max_error>ego.max_error*3,"hidden lag counterexample absent");
    require(lag_correct.fits>300 && lag_correct.max_error<lag_zero.max_error/2,"configured IMU compensation did not improve center");
    (void)lag_wrong;(void)all_zero;
    require(accel.rejected>ego.rejected,"acceleration rejection absent");
    // Preserve this availability failure without relaxing the 25 mm firing gate.
    require(holdout.fits>300 && holdout.shots==0,"noisy holdout counterexample changed");
    require(mild.fits>300 && mild.shots>50,"mild noise produced no useful model");
    require(faults.fits>300 && faults.rejected>ego.rejected,"dropout/outlier recovery failed");
    (void)fast;
    Tracker invalid(pi/2,Eigen::Matrix3d::Identity());
    Frame f{1,1.01,1.02,{1,Eigen::Quaterniond::Identity()},{{{4,0,.4},{-1,0,0}}}};
    invalid.update(f);require(!invalid.update(f),"duplicate timestamp accepted");
    f.image_host_stamp_s=1.01;f.visual_imu.published_s=.5;
    require(!invalid.update(f),"stale IMU accepted");
    std::cout<<"relative aim checks passed; artifacts: "<<out.string()<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
