#include "rm_tracker/body_ekf.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace dt46_vision {
namespace { constexpr double pi=3.14159265358979323846;
double wrap(double a){return std::remainder(a,2*pi);}
Eigen::Matrix4d noise(double sigma,double yaw){
    Eigen::Matrix4d r=Eigen::Matrix4d::Identity()*sigma*sigma;r(3,3)=yaw;return r;
}
}
void BodyEkf::init(const State& state,const Covariance& covariance){
    x_=state;p_=covariance;accepted_=0;last_nis_=0;
}
void BodyEkf::set_limits(double minimum,double maximum,double distance,double yaw){
    if(!std::isfinite(minimum+maximum+distance+yaw)||minimum<=0||maximum<minimum||distance<=0||yaw<=0)
        throw std::invalid_argument("invalid body geometry/association limits");
    radius_min_=minimum;radius_max_=maximum;match_distance_=distance;match_yaw_=yaw;
}
void BodyEkf::predict(double dt,double q_xyz,double q_yaw,double q_radius){
    if(!std::isfinite(dt)||dt<0||!std::isfinite(q_xyz+q_yaw+q_radius)||q_xyz<0||q_yaw<0||q_radius<0)
        throw std::invalid_argument("invalid body propagation interval or noise");
    Covariance f=Covariance::Identity(),q=Covariance::Zero();
    for(int i: {0,2,4,6}){
        f(i,i+1)=dt;const double density=i==6?q_yaw:q_xyz;
        q(i,i)=density*dt*dt*dt/3;q(i,i+1)=q(i+1,i)=density*dt*dt/2;q(i+1,i+1)=density*dt;
    }
    for(int i:{8,9,10})q(i,i)=q_radius*dt;
    x_=f*x_;x_[6]=wrap(x_[6]);p_=f*p_*f.transpose()+q;p_=(.5*(p_+p_.transpose())).eval();
}
Eigen::Vector4d BodyEkf::plate(int index,double ahead) const{
    const int group=index%2;const double yaw=x_[6]+index*pi/2+x_[7]*ahead;
    return {x_[0]+x_[1]*ahead-x_[8+group]*std::cos(yaw),
        x_[2]+x_[3]*ahead-x_[8+group]*std::sin(yaw),x_[4]+x_[5]*ahead+group*x_[10],wrap(yaw)};
}
Eigen::Matrix<double,4,11> BodyEkf::jacobian(int index) const{
    const int group=index%2;const double yaw=x_[6]+index*pi/2,r=x_[8+group];
    Eigen::Matrix<double,4,11> h=Eigen::Matrix<double,4,11>::Zero();
    h(0,0)=h(1,2)=h(2,4)=h(3,6)=1;h(0,6)=r*std::sin(yaw);h(1,6)=-r*std::cos(yaw);
    h(0,8+group)=-std::cos(yaw);h(1,8+group)=-std::sin(yaw);h(2,10)=group;return h;
}
bool BodyEkf::correct(const Observation& obs,int index,double sigma,double yaw_variance){
    const auto h=jacobian(index);const auto predicted=plate(index);
    Eigen::Vector4d residual;residual.head<3>()=obs.position-predicted.head<3>();residual[3]=wrap(obs.yaw-predicted[3]);
    const auto r=noise(sigma,yaw_variance);
    const Eigen::Matrix4d s=h*p_*h.transpose()+r;const auto decomposition=s.ldlt();
    if(decomposition.info()!=Eigen::Success||!decomposition.isPositive())return false;
    last_nis_=residual.dot(decomposition.solve(residual));
    if(!std::isfinite(last_nis_)||last_nis_>18.47)return false;
    const Eigen::Matrix<double,11,4> k=decomposition.solve((p_*h.transpose()).transpose()).transpose();
    State next=x_+k*residual;next[6]=wrap(next[6]);
    if(!next.allFinite()||next[8]<radius_min_||next[8]>radius_max_||next[9]<radius_min_||next[9]>radius_max_||std::abs(next[10])>.3)return false;
    const Covariance a=Covariance::Identity()-k*h;
    Covariance covariance=a*p_*a.transpose()+k*r*k.transpose();
    covariance=(.5*(covariance+covariance.transpose())).eval();
    if(!covariance.allFinite())return false;
    x_=next;p_=covariance;++accepted_;return true;
}
int BodyEkf::update(const std::vector<Observation>& observations,double sigma,double yaw_variance){
    if(!std::isfinite(sigma)||!std::isfinite(yaw_variance)||sigma<=0||yaw_variance<=0)return -1;
    struct Match{int observation,index;double score;};std::vector<Match> matches;
    // Every association uses the same predicted body; duplicate assignments cannot update it twice.
    for(int j=0;j<int(observations.size())&&j<16;++j){
        const auto& obs=observations[j];if(!obs.position.allFinite()||!std::isfinite(obs.yaw))continue;
        Match best{j,-1,1e100};double second=1e100;
        for(int i=0;i<4;++i){
            const auto prediction=plate(i);const auto h=jacobian(i);
            Eigen::Vector4d residual;residual.head<3>()=obs.position-prediction.head<3>();residual[3]=wrap(obs.yaw-prediction[3]);
            // PnP mirror ambiguity is resolved using the same geometric branch as the position.
            if(std::abs(residual[3])>pi/2)residual[3]=wrap(residual[3]+pi);
            if(residual.head<3>().norm()>match_distance_||std::abs(residual[3])>match_yaw_)continue;
            const Eigen::Matrix4d s=h*p_*h.transpose()+noise(sigma,yaw_variance);const auto ldlt=s.ldlt();
            if(ldlt.info()!=Eigen::Success||!ldlt.isPositive())continue;
            const double cost=residual.dot(ldlt.solve(residual));
            if(cost<best.score){second=best.score;best={j,i,cost};}else second=std::min(second,cost);
        }
        if(best.index>=0&&best.score<18.47&&second-best.score>1)matches.push_back(best);
    }
    std::sort(matches.begin(),matches.end(),[](const Match& a,const Match& b){return a.score<b.score;});
    std::array<bool,4> used{};int selected=-1;
    for(const auto& match:matches){
        if(used[match.index])continue;
        auto obs=observations[match.observation];
        if(std::abs(wrap(obs.yaw-plate(match.index)[3]))>pi/2)obs.yaw=wrap(obs.yaw+pi);
        if(correct(obs,match.index,sigma,yaw_variance)){used[match.index]=true;if(selected<0)selected=match.index;}
    }
    return selected;
}
bool BodyEkf::ready() const{
    return accepted_>=8&&p_(0,0)+p_(2,2)<.005&&p_(8,8)<.0025&&p_(9,9)<.0025;
}
}
