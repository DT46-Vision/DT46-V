// Reuses only the independent RK4 reference from the regression test.
#define main regression_main
#include "../../tests/ballistics_check.cpp"
#undef main
#include "baseline_ballistics.hpp"
#include <mujoco/mujoco.h>
#include <fstream>
#include <vector>

void passive_drag(const mjModel* m, mjData* d) {
    const double speed=std::sqrt(d->qvel[0]*d->qvel[0]+d->qvel[1]*d->qvel[1]+d->qvel[2]*d->qvel[2]);
    for(int i=0;i<3;++i)d->qfrc_passive[i]-=.019*m->body_mass[1]*speed*d->qvel[i];
}
std::array<double,2> mujoco_reference(const mjModel* model,mjData* data,double range,double pitch,double speed){
    mj_resetData(model,data);
    data->qvel[0]=speed*std::cos(pitch);data->qvel[2]=speed*std::sin(pitch);
    for(int i=0;i<3000;++i){
        const double x=data->qpos[0],z=data->qpos[2],t=data->time;
        mj_step(model,data);
        if(data->qpos[0]>=range){
            const double f=(range-x)/(data->qpos[0]-x);
            return {z+f*(data->qpos[2]-z),t+f*(data->time-t)};
        }
    }
    throw std::runtime_error("MuJoCo did not reach range");
}
int main(int argc,char** argv){
    if(argc!=2)return 2;
    std::ofstream csv(std::string(argv[1])+"/ballistic_grid.csv");
    csv<<"speed,range,height,old_error_m,new_error_m,old_tof_error_s,new_tof_error_s\n";
    Baseline baseline;double old_max=0,new_max=0,new_time_max=0;int unreachable=0,cases=0;
    for(int speed=20;speed<=30;++speed){
        baseline.bullet_speed=speed;baseline.build_ballistic_lut();
        for(int ir=0;ir<49;++ir)for(int iz=-14;iz<=14;++iz){
            const double range=ir==0?.2:ir==1?.3:.5+(ir-2)*.25,height=iz*.25;
            const auto s=solve_projectile(range,height,speed,.019);
            const auto actual=reference(range,s.pitch,speed,.019);
            const double new_error=actual[0]-height,new_time=actual[1]-s.flight_time;
            const double old_pitch=baseline.lut_lookup(range,height);
            double old_error=std::numeric_limits<double>::quiet_NaN(),old_time=old_error;
            if(std::abs(old_pitch)<1.5707){
                try{const auto previous=reference(range,old_pitch,speed,.019);old_error=previous[0]-height;
                    old_time=previous[1]-baseline.lut_lookup_tof(range,height);}
                catch(const std::exception&){++unreachable;}
            }else ++unreachable;
            old_max=std::max(old_max,std::abs(old_error));new_max=std::max(new_max,std::abs(new_error));
            new_time_max=std::max(new_time_max,std::abs(new_time));++cases;
            csv<<speed<<','<<range<<','<<height<<','<<old_error<<','<<new_error<<','<<old_time<<','<<new_time<<'\n';
        }
    }
    const char* xml="<mujoco><option timestep='0.001' integrator='RK4' gravity='0 0 -9.81'/><worldbody><body><freejoint/><geom type='sphere' size='0.0085' mass='0.0032' contype='0' conaffinity='0'/></body></worldbody></mujoco>";
    mjVFS vfs; mj_defaultVFS(&vfs);mj_addBufferVFS(&vfs,"audit.xml",xml,int(std::char_traits<char>::length(xml)));
    char error[1024]{};mjModel* model=mj_loadXML("audit.xml",&vfs,error,sizeof(error));mj_deleteVFS(&vfs);
    if(!model)throw std::runtime_error(error);
    mjData* data=mj_makeData(model);mjcb_passive=passive_drag;
    double mj_max=0,mj_time=0,mj_sum_us=0;int mj_cases=0;
    for(double speed:{20.,25.,30.})for(double range:{.5,3.,5.,12.})for(double height:{-3.5,0.,3.5}){
        const auto s=solve_projectile(range,height,speed,.019);
        const auto expected=reference(range,s.pitch,speed,.019);
        const auto begin=std::chrono::steady_clock::now();
        const auto actual=mujoco_reference(model,data,range,s.pitch,speed);
        mj_sum_us+=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
        mj_max=std::max(mj_max,std::abs(actual[0]-expected[0]));mj_time=std::max(mj_time,std::abs(actual[1]-expected[1]));++mj_cases;
    }
    mj_deleteData(data);mj_deleteModel(model);mjcb_passive=nullptr;
    std::ofstream out(std::string(argv[1])+"/ballistics_audit.json");
    out<<"{\n  \"cases\":"<<cases<<",\n  \"old_max_height_error_m\":"<<old_max
       <<",\n  \"old_unreachable_cases\":"<<unreachable<<",\n  \"new_max_height_error_m\":"<<new_max
       <<",\n  \"new_max_time_error_s\":"<<new_time_max<<",\n  \"mujoco_version\":\""<<mj_versionString()
       <<"\",\n  \"mujoco_cases\":"<<mj_cases<<",\n  \"mujoco_vs_rk4_max_height_m\":"<<mj_max
       <<",\n  \"mujoco_vs_rk4_max_time_s\":"<<mj_time<<",\n  \"mujoco_forward_mean_us\":"<<mj_sum_us/mj_cases<<"\n}\n";
    std::cout<<"grid="<<cases<<" old_max="<<old_max<<" new_max="<<new_max<<" old_unreachable="<<unreachable
             <<" MuJoCo="<<mj_versionString()<<" forward_us="<<mj_sum_us/mj_cases<<" reference_delta_m="<<mj_max<<'\n';
    return mj_max<.00001&&new_max<.01?0:1;
}
