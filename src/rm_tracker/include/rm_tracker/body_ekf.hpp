#pragma once
#include <Eigen/Dense>
#include <vector>

namespace dt46_vision {
// One body: cx,vx,cy,vy,zA,vz,theta,omega,rA,rB,hB. zA fixes the vertical gauge.
class BodyEkf {
public:
    using State=Eigen::Matrix<double,11,1>;
    using Covariance=Eigen::Matrix<double,11,11>;
    struct Observation { Eigen::Vector3d position; double yaw; };
    void init(const State& state,const Covariance& covariance);
    void set_limits(double radius_min,double radius_max,double match_distance,double match_yaw);
    void predict(double dt,double q_xyz,double q_yaw,double q_radius);
    int update(const std::vector<Observation>& observations,double position_sigma,double yaw_variance,double depth_sigma=0);
    static Eigen::Matrix4d measurement_covariance(const Eigen::Vector3d& position,double sigma,double yaw_variance,double depth_sigma=0);
    const State& state() const { return x_; }
    const Covariance& covariance() const { return p_; }
    Eigen::Vector4d plate(int index,double ahead=0) const;
    Eigen::Matrix<double,4,11> jacobian(int index) const;
    BodyEkf forecast(double ahead) const;
    Eigen::Matrix3d plate_covariance(int index) const;
    bool established() const { return accepted_>=8; }
    unsigned matched_mask() const { return matched_mask_; }
    const char* rejection_reason() const { return rejection_reason_; }
    bool ready() const;
    double last_nis() const { return last_nis_; }
    double association_cost() const { return association_cost_; }
private:
    State x_=State::Zero();
    Covariance p_=Covariance::Identity();
    int accepted_=0;
    double last_nis_=0;
    double association_cost_=50;
    unsigned matched_mask_=0;
    double q_xyz_=0,q_yaw_=0,q_radius_=0;
    const char* rejection_reason_="no_observations";
    double radius_min_=.1,radius_max_=.5,match_distance_=.4,match_yaw_=.8;
    bool correct(const Observation& obs,int index,double sigma,double yaw_variance,double depth_sigma);
};
}
