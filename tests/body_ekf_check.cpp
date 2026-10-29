#include "rm_tracker/body_ekf.hpp"
#include "rm_tracker/frame_clock.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>

using namespace dt46_vision;
void require(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
BodyEkf::State truth(double t,double acceleration=0){
    BodyEkf::State x; x<<4+.35*t+.5*acceleration*t*t,.35+acceleration*t,.2-.12*t,-.12,.45,0,
        std::remainder(.3+3*t,2*3.141592653589793),3,.22,.32,.07;return x;
}
BodyEkf initial(){
    BodyEkf sensor_model;sensor_model.init(truth(0),BodyEkf::Covariance::Identity());
    const auto observed=sensor_model.plate(0);
    BodyEkf f;auto x=BodyEkf::State::Zero().eval();
    x[0]=observed[0]+.26*std::cos(observed[3]);x[2]=observed[1]+.26*std::sin(observed[3]);
    x[4]=observed[2];x[6]=observed[3];x[8]=x[9]=.26;
    BodyEkf::Covariance p=BodyEkf::Covariance::Identity();p(8,8)=p(9,9)=p(10,10)=.01;f.init(x,p);return f;
}
int main(int argc,char** argv){try{
    std::filesystem::path out=argc>1?argv[1]:"body_ekf_results";std::filesystem::create_directories(out);
    std::ofstream csv(out/"frames.csv");csv<<"frame,time_s,center_error_m,rA,rB,hB,omega,matched,ready\n";
    FrameClock clock;require(clock.advance(0).reset,"first frame not initialized");
    const auto tiny=clock.advance(.0005);require(tiny.accepted&&std::abs(tiny.dt-.0005)<1e-12,"tiny interval fabricated");
    require(!clock.advance(.0005).accepted&&!clock.advance(-1).accepted,"duplicate/backward accepted");
    const auto longish=clock.advance(.2005);require(!longish.reset&&std::abs(longish.dt-.2)<1e-12,"200 ms clipped");
    require(clock.advance(1).reset,"long gap failed to reset");
    auto single=initial(),split=single;
    single.predict(.2,2.5,15,.005);for(int i=0;i<20;++i)split.predict(.01,2.5,15,.005);
    require((single.state()-split.state()).norm()<1e-10,"state propagation not time consistent");
    require((single.covariance()-split.covariance()).norm()<1e-10,"Q does not cover real interval");
    auto f=initial();
    for(int index=0;index<4;++index){
        const auto h=f.jacobian(index);const auto base=f.state();
        for(int j=0;j<11;++j){auto plus=base,minus=base;plus[j]+=1e-6;minus[j]-=1e-6;
            BodyEkf a,b;a.init(plus,f.covariance());b.init(minus,f.covariance());
            require(((a.plate(index)-b.plate(index))/(2e-6)-h.col(j)).norm()<1e-6,"body Jacobian wrong");}
    }
    std::mt19937 random(8146);std::normal_distribution<double> noise(0,.002);
    double t=0,sum=0,maxerr=0;int count=0,matched=0;
    for(int frame=0;frame<1200;++frame){
        const double dt=frame%23==0?.04:(frame%17==0?.0005:.01);t+=dt;
        BodyEkf actual;actual.init(truth(t),BodyEkf::Covariance::Identity());
        f.predict(dt,2.5,15,.0001);std::vector<BodyEkf::Observation> observations;
        // Alternate all four anonymous plates; group geometry is never given to the estimator.
        for(int j=0;j<2;++j){const auto p=actual.plate((frame/9+j)%4);Eigen::Vector3d position=p.head<3>();
            for(int k=0;k<3;++k)position[k]+=noise(random);
            observations.push_back({position,p[3]+noise(random)});}
        if(frame==400)observations[0].position.x()+=3;
        if(frame>=700&&frame<710)observations.clear();
        const int index=f.update(observations,.008,.0025);if(index>=0)++matched;
        const double err=(f.state().head<4>()-truth(t).head<4>()).norm();
        const double center_err=std::hypot(f.state()[0]-truth(t)[0],f.state()[2]-truth(t)[2]);
        if(frame>100){sum+=center_err*center_err;maxerr=std::max(maxerr,center_err);++count;}
        require(f.covariance().allFinite()&&(f.covariance()-f.covariance().transpose()).norm()<1e-10,"covariance asymmetric");
        Eigen::SelfAdjointEigenSolver<BodyEkf::Covariance> eig(f.covariance());
        require(eig.eigenvalues().minCoeff()>-1e-10,"covariance not PSD");
        const auto p0=f.plate(0),p2=f.plate(2);require((p0.head<2>()+p2.head<2>()-2*Eigen::Vector2d(f.state()[0],f.state()[2])).norm()<1e-10,"shared center broken");
        require(std::abs(p0.z()-p2.z())<1e-12,"opposite height changed");
        csv<<frame<<','<<t<<','<<center_err<<','<<f.state()[8]<<','<<f.state()[9]<<','<<f.state()[10]<<','<<f.state()[7]<<','<<index<<','<<f.ready()<<'\n';
        (void)err;
    }
    const double rmse=std::sqrt(sum/count);
    require(matched>1100&&rmse<.015&&maxerr<.04,"shared center continuity failed");
    require(std::abs(f.state()[8]-.22)<.015&&std::abs(f.state()[9]-.32)<.015&&std::abs(f.state()[10]-.07)<.01,"independent geometry failed");
    auto before=f.state();auto covariance=f.covariance();
    require(f.update({{{100,100,100},0}},.008,.0025)<0,"outlier accepted");
    require((f.state()-before).norm()==0&&(f.covariance()-covariance).norm()==0,"rejected measurement changed state");
    require(f.ready(),"geometry never became ready");
    auto mirrored=f,ordinary=f;
    const auto observed=f.plate(0);
    ordinary.update({{observed.head<3>(),observed[3]}},.008,.0025);
    mirrored.update({{observed.head<3>(),observed[3]+3.141592653589793}},.008,.0025);
    require((ordinary.state()-mirrored.state()).norm()<1e-8,"mirror branch inconsistent");
    auto duplicate=f,once=f;
    once.update({{observed.head<3>(),observed[3]}},.008,.0025);
    duplicate.update({{observed.head<3>(),observed[3]},{observed.head<3>(),observed[3]}},.008,.0025);
    require((duplicate.covariance()-once.covariance()).norm()<1e-10,"duplicate observation double counted");
    auto accelerated=initial();double accel_sum=0,accel_max=0;
    for(int i=1;i<=300;++i){const double time=i*.01;BodyEkf actual;actual.init(truth(time,3),BodyEkf::Covariance::Identity());
        accelerated.predict(.01,2.5,15,.0001);std::vector<BodyEkf::Observation> observations;
        for(int j=0;j<2;++j){const auto p=actual.plate((i/9+j)%4);observations.push_back({p.head<3>(),p[3]});}
        accelerated.update(observations,.008,.0025);
        if(i>100){const double e=std::hypot(accelerated.state()[0]-truth(time,3)[0],accelerated.state()[2]-truth(time,3)[2]);accel_sum+=e*e;accel_max=std::max(accel_max,e);}}
    require(std::sqrt(accel_sum/200)<.025,"relative acceleration tracking failed");
    std::ofstream summary(out/"summary.txt");summary<<"frames=1200\nmatched="<<matched<<"\ncenter_rmse_m="<<rmse<<"\ncenter_max_m="<<maxerr<<"\nrA="<<f.state()[8]<<"\nrB="<<f.state()[9]<<"\nhB="<<f.state()[10]<<'\n';
    summary<<"acceleration_center_rmse_m="<<std::sqrt(accel_sum/200)<<"\nacceleration_center_max_m="<<accel_max<<'\n';
    std::cout<<"shared body, geometry, Jacobians, Joseph PSD, real dt checks passed; center RMSE "<<rmse<<" m, max "<<maxerr<<" m\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
