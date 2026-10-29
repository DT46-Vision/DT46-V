#include "relative_aim.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace dt46::relative {
namespace {
constexpr double pi = std::numbers::pi;
double wrap(double x) { return std::remainder(x, 2*pi); }
double offset(int i, double adjacent) { return (i/2)*pi + (i%2)*adjacent; }
}
Tracker::Tracker(double adjacent, Eigen::Matrix3d mounting, DelayConfig delays)
    : adjacent_(adjacent), mounting_(std::move(mounting)), delays_(delays) {
    if (!(adjacent > .2 && adjacent < pi-.2) || !mounting_.allFinite() ||
        (mounting_.transpose()*mounting_-Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
        std::abs(mounting_.determinant()-1) > 1e-6 ||
        !std::isfinite(delays_.image_stamp_delay_s) || delays_.image_stamp_delay_s<0 ||
        !std::isfinite(delays_.imu_publish_delay_s) || delays_.imu_publish_delay_s<0)
        throw std::invalid_argument("invalid phase or static mounting");
}
bool Tracker::update(const Frame& f) {
    body_.reset();
    const double t = f.image_host_stamp_s-delays_.image_stamp_delay_s;
    const double qnorm = f.visual_imu.orientation.norm();
    if (!std::isfinite(t) || !std::isfinite(f.received_s) || !std::isfinite(f.detection_done_s) ||
        !std::isfinite(f.visual_imu.published_s) || !std::isfinite(qnorm) ||
        std::abs(qnorm-1) > 1e-6 || f.received_s < f.image_host_stamp_s || f.detection_done_s < f.received_s ||
        t <= last_stamp_) {
        reason_ = "invalid_host_time_or_imu"; return false;
    }
    if (last_stamp_ >= 0 && t-last_stamp_ > .15) { samples_.clear(); omega_=0; }
    if (f.plates.empty()) { reason_="no_observation"; return false; }
    const double imu_time=f.visual_imu.published_s-delays_.imu_publish_delay_s;
    const double extrapolation=t-imu_time;
    auto orientation=f.visual_imu.orientation;
    const auto previous=previous_imu_;previous_imu_=f.visual_imu;
    if(std::abs(extrapolation)>.12){reason_="imu_prediction_horizon";return false;}
    if(std::abs(extrapolation)>1e-6){
        if(!previous){reason_="imu_rate_unavailable";return false;}
        const double dt=f.visual_imu.published_s-previous->published_s;
        if(dt<=0 || dt>.15){reason_="imu_rate_unavailable";return false;}
        // Direct constant-angular-rate extrapolation; host delay configuration is not synchronization.
        const Eigen::Quaterniond delta=previous->orientation.conjugate()*orientation;
        orientation=(orientation*Eigen::Quaterniond::Identity().slerp(extrapolation/dt,delta)).normalized();
    }
    const auto rotation = (orientation.toRotationMatrix()*mounting_).eval();
    std::vector<Sample> incoming;
    std::array<bool,4> used{};
    const double expected = phase_ + omega_*(last_stamp_ < 0 ? 0 : t-last_stamp_);
    double phase_sum = 0;
    for (const auto& p : f.plates) {
        if (!p.camera_position_m.allFinite() || !p.camera_normal.allFinite() ||
            std::abs(p.camera_normal.norm()-1) > .01) { reason_="invalid_plate"; return false; }
        const Vec3 n=rotation*p.camera_normal;
        if (n.head<2>().norm()<.5) { reason_="nonvertical_body"; return false; }
        const double yaw=std::atan2(n.y(),n.x());
        if (samples_.empty() && incoming.empty()) phase_=yaw;
        const double reference = samples_.empty() ? phase_ : expected;
        int best=0; double error=10, second=10;
        for (int i=0;i<4;++i) {
            const double e=std::abs(wrap(yaw-reference-offset(i,adjacent_)));
            if(e<error){second=error;error=e;best=i;}else second=std::min(second,e);
        }
        if(error>.45 || second-error<.15 || used[best]) { reason_="ambiguous_association"; return false; }
        used[best]=true;
        const double phase=reference+wrap(yaw-offset(best,adjacent_)-reference);
        phase_sum+=phase;
        incoming.push_back({t,phase,yaw,best%2,rotation*p.camera_position_m});
    }
    phase_=phase_sum/incoming.size(); last_stamp_=t;
    for (const auto& s:incoming) samples_.push_back(s);
    while (!samples_.empty() && (t-samples_.front().t>.6 || samples_.size()>160)) samples_.pop_front();
    return fit(t);
}
bool Tracker::fit(double t) {
    const int n=int(samples_.size());
    if(n<12 || t-samples_.front().t<.15) {reason_="insufficient_history";return false;}
    Eigen::MatrixXd a=Eigen::MatrixXd::Zero(2*n,6), z=Eigen::MatrixXd::Zero(n,3), angle(n,2);
    Eigen::VectorXd b(2*n), bz(n), ba(n);
    for(int j=0;j<n;++j) {
        const auto& s=samples_[j]; const double dt=s.t-t;
        const double yaw=s.yaw;
        a(2*j,0)=a(2*j+1,1)=1; a(2*j,2)=a(2*j+1,3)=dt;
        a(2*j,4+s.group)=std::cos(yaw); a(2*j+1,4+s.group)=std::sin(yaw);
        b.segment<2>(2*j)=s.position.head<2>();
        z(j,0)=1;z(j,1)=dt;z(j,2)=s.group;bz[j]=s.position.z();
        angle(j,0)=1;angle(j,1)=dt;ba[j]=s.phase;
    }
    auto qr=a.colPivHouseholderQr(); qr.setThreshold(1e-5);
    auto qz=z.colPivHouseholderQr();qz.setThreshold(1e-5);
    if(qr.rank()!=6 || qz.rank()!=3){reason_="unobservable_geometry";return false;}
    const Eigen::VectorXd x=qr.solve(b), h=qz.solve(bz), w=angle.colPivHouseholderQr().solve(ba);
    const double residual=std::sqrt(((a*x-b).squaredNorm()+(z*h-bz).squaredNorm())/n);
    if(!x.allFinite() || !h.allFinite() || !w.allFinite() || residual>.025 ||
        (angle*w-ba).norm()/std::sqrt(double(n))>.04 ||
        x[4]<.1 || x[4]>.5 || x[5]<.1 || x[5]>.5) {reason_="model_inconsistent";return false;}
    Body body;body.stamp_s=t;body.center={x[0],x[1],h[0]};body.velocity={x[2],x[3],h[1]};
    body.radius={x[4],x[5]};body.height={0,h[2]};body.phase=w[0];body.omega=w[1];body.residual_m=residual;
    body_=body;omega_=body.omega;reason_="fit";return true;
}
Vec3 Tracker::plate(int i,double ahead) const {
    const auto& b=body_.value();const double yaw=b.phase+b.omega*ahead+offset(i,adjacent_);
    return b.center+b.velocity*ahead+Vec3(b.radius[i%2]*std::cos(yaw),b.radius[i%2]*std::sin(yaw),b.height[i%2]);
}
Shot aim(const Tracker& tracker,const Timing& timing,double speed,double speed_stamp,const Vec3& bore) {
    Shot best;
    if(!tracker.body()) return best;
    const auto& b=*tracker.body();
    if(!std::isfinite(timing.solve_s) || !std::isfinite(speed_stamp) || !std::isfinite(speed) ||
        speed<10 || speed>50 || speed_stamp>timing.solve_s || timing.solve_s-speed_stamp>.5 ||
        timing.solve_s<b.stamp_s || timing.solve_s-b.stamp_s>.1 || !bore.allFinite() || std::abs(bore.norm()-1)>.01) {
        best.reason="stale_or_invalid_input";return best;
    }
    if(!timing.stamp_error_bound_s || !timing.execution_delay_s) {best.reason="unknown_timing";return best;}
    const double uncertainty=*timing.stamp_error_bound_s, delay=*timing.execution_delay_s;
    if(!std::isfinite(uncertainty)||!std::isfinite(delay)||uncertainty<0||delay<0||delay>.1) {best.reason="invalid_timing";return best;}
    const double age=timing.solve_s-b.stamp_s+delay;
    const double horizon=age+b.center.norm()/speed;
    double best_score=std::numeric_limits<double>::infinity();
    for(int i=0;i<4;++i) {
        const Vec3 p=tracker.plate(i,horizon);
        const double d=p.head<2>().norm(), g=9.81, v2=speed*speed;
        const double disc=v2*v2-g*(g*d*d+2*p.z()*v2);
        if(d<.1 || disc<=0) continue;
        const double pitch=std::atan((v2-std::sqrt(disc))/(g*d));
        const double flight=d/(speed*std::cos(pitch));
        const Vec3 actual=tracker.plate(i,age+flight);
        const Vec3 normal=(p-(b.center+b.velocity*horizon)).cwiseProduct(Vec3(1,1,0)).normalized();
        if(normal.dot(-p.normalized())<.3) continue;
        const double yaw=std::atan2(p.y(),p.x());
        const Vec3 direction(std::cos(pitch)*std::cos(yaw),std::cos(pitch)*std::sin(yaw),std::sin(pitch));
        const double pointing=std::acos(std::clamp(direction.dot(bore),-1.,1.))*p.norm();
        const double miss=(actual-p).norm()+uncertainty*(b.velocity.norm()+std::abs(b.omega)*b.radius[i%2])+3*b.residual_m;
        const double score=pointing+miss;
        if(score<best_score) {
            best_score=score;best={true,score<.025,i,yaw,pitch,flight,miss,score<.025?"conditional_fire":"miss_budget",p};
        }
    }
    if(!best.valid)best.reason="no_visible_ballistic_solution";
    return best;
}
}
