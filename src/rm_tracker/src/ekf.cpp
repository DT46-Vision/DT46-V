#include "rm_tracker/ekf.hpp"

namespace dt46_vision {

ExtendedKalmanFilter::ExtendedKalmanFilter() : is_initialized_(false) {
    X_.setZero();
    P_.setIdentity();
    F_.setIdentity(); // 恒定模型基座，后续只更新含 dt 的非对角线元素
    Q_.setZero();
    R_.setZero();

    // 给定初始安全参数
    s2qxyz_ = 20.0;
    s2qyaw_ = 100.0;
    s2qr_   = 800.0;
    r_xyz_factor_ = 0.05;
    r_yaw_ = 0.02;
    stable_dist_ = 1.5;
}

void ExtendedKalmanFilter::init(const Eigen::Matrix<double, 9, 1>& init_x, const Eigen::Matrix<double, 9, 9>& init_p) {
    X_ = init_x;
    P_ = init_p;
    X_(6) = normalizeAngle(X_(6));
    is_initialized_ = true;
}

void ExtendedKalmanFilter::init_QR(double q_xyz, double q_yaw, double q_r, double r_xyz_factor, double r_yaw, double stable_dist) {
    s2qxyz_ = q_xyz;
    s2qyaw_ = q_yaw;
    s2qr_   = q_r;
    r_xyz_factor_ = r_xyz_factor;
    r_yaw_ = r_yaw;
    stable_dist_ = stable_dist;
}

void ExtendedKalmanFilter::predict(double dt) {
    if (!is_initialized_) return;

    // 1. 更新状态转移矩阵 F (只更新动态部分)
    F_(0, 1) = dt; // x  += vx * dt
    F_(2, 3) = dt; // y  += vy * dt
    F_(4, 5) = dt; // z  += vz * dt
    F_(6, 7) = dt; // yaw+= vyaw * dt

    // 2. 动态构建过程噪声矩阵 Q
    Q_.setZero();
    double t2 = dt * dt;
    double t3 = t2 * dt;
    double t4 = t3 * dt;

    // XYZ 噪声分量
    double q_xyz_x  = (t4 / 4.0) * s2qxyz_;
    double q_xyz_vx = (t3 / 2.0) * s2qxyz_;
    double q_xyz_vv = t2 * s2qxyz_;

    Q_(0,0) = q_xyz_x; Q_(0,1) = q_xyz_vx;
    Q_(1,0) = q_xyz_vx; Q_(1,1) = q_xyz_vv;
    Q_(2,2) = q_xyz_x; Q_(2,3) = q_xyz_vx;
    Q_(3,2) = q_xyz_vx; Q_(3,3) = q_xyz_vv;
    Q_(4,4) = q_xyz_x; Q_(4,5) = q_xyz_vx;
    Q_(5,4) = q_xyz_vx; Q_(5,5) = q_xyz_vv;

    // Yaw 噪声分量
    double q_yaw_x  = (t4 / 4.0) * s2qyaw_;
    double q_yaw_vx = (t3 / 2.0) * s2qyaw_;
    double q_yaw_vv = t2 * s2qyaw_;

    Q_(6,6) = q_yaw_x; Q_(6,7) = q_yaw_vx;
    Q_(7,6) = q_yaw_vx; Q_(7,7) = q_yaw_vv;

    // 半径噪声分量
    Q_(8,8) = t2 * s2qr_;

    // 3. 预测步：先验状态与协方差更新
    X_ = F_ * X_;
    X_(6) = normalizeAngle(X_(6));
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void ExtendedKalmanFilter::update(const Eigen::Vector4d& measurement) {
    if (!is_initialized_) return;

    double xc  = X_(0);
    double yc  = X_(2);
    double za  = X_(4);
    double yaw = X_(6);
    double r   = X_(8);

    double s_yaw = std::sin(yaw);
    double c_yaw = std::cos(yaw);

    // 1. 动态构建观测雅可比矩阵 H (4x9)
    Eigen::Matrix<double, 4, 9> H = Eigen::Matrix<double, 4, 9>::Zero();

    // Row 0: 对 x_m 的偏导
    H(0, 0) = 1.0;
    H(0, 6) = r * s_yaw;
    H(0, 8) = -c_yaw;

    // Row 1: 对 y_m 的偏导
    H(1, 2) = 1.0;
    H(1, 6) = -r * c_yaw;
    H(1, 8) = -s_yaw;

    // Row 2: 对 z_m 的偏导
    H(2, 4) = 1.0;

    // Row 3: 对 yaw_m 的偏导
    H(3, 6) = 1.0;

    // 2. 动态更新观测噪声 R
    double obs_x = measurement(0), obs_y = measurement(1), obs_z = measurement(2);
    double dist_h = std::hypot(obs_x, obs_y);
    double base_noise = 0.05;

    R_.setZero();
    R_(0,0) = std::abs(r_xyz_factor_ * obs_x) + base_noise;
    R_(1,1) = std::abs(r_xyz_factor_ * obs_y) + base_noise;
    R_(2,2) = std::abs(r_xyz_factor_ * obs_z) + base_noise;

    // 远距离 Yaw 角信任降级
    if (dist_h > stable_dist_) {
        double dynamic_r_yaw = r_yaw_ * std::pow(dist_h / 1.5, 2.0);
        R_(3,3) = std::min(dynamic_r_yaw, 10.0);
    } else {
        R_(3,3) = r_yaw_;
    }

    // 3. 计算预计观测值 h(x) 与残差 Y
    Eigen::Vector4d Z_pred(xc - r * c_yaw, yc - r * s_yaw, za, yaw);
    Eigen::Vector4d Y = measurement - Z_pred;
    Y(3) = normalizeAngle(Y(3)); // 核心：残差中的角度差必须被归一化！

    // 4. 计算残差协方差 S (Innovation Covariance)
    Eigen::Matrix<double, 4, 4> S = H * P_ * H.transpose() + R_;

    // 使用 LDLT 分解代替直接求逆（优化）
    Eigen::LDLT<Eigen::Matrix<double, 4, 4>> ldlt(S);
    Eigen::Matrix<double, 4, 4> S_inv = ldlt.solve(Eigen::Matrix<double, 4, 4>::Identity());

    // 5. 【马氏距离野值剔除】自由度为 4 的卡方分布
    double mahalanobis_sq = (Y.transpose() * ldlt.solve(Y)).value();
    if (mahalanobis_sq > 30.0) {
        // 如果测量值离群严重，拒绝更新，直接信任当前的预测步
        return;
    }

    // 6. 计算卡尔曼增益 K (9x4)
    Eigen::Matrix<double, 9, 4> K = P_ * H.transpose() * S_inv;

    // 7. 更新后验状态 X
    X_ = X_ + K * Y;
    X_(6) = normalizeAngle(X_(6));

    // 8. Joseph 形式更新协方差 P (保证数值稳定性和正定性)
    Eigen::Matrix<double, 9, 9> I = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 9, 9> I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();
}

void ExtendedKalmanFilter::smooth_reset_covariance() {
    // 适度放大位置和角度的方差，让滤波器在跳变后几帧稍微更信任观测
    P_(0, 0) += 0.05;
    P_(2, 2) += 0.05;
    P_(4, 4) += 0.05;
    P_(6, 6) += 0.2;
    P_(8, 8) += 0.01;
}

void ExtendedKalmanFilter::setState(const Eigen::Matrix<double, 9, 1>& state) {
    X_ = state;
    X_(6) = normalizeAngle(X_(6));
}

Eigen::Matrix<double, 9, 1> ExtendedKalmanFilter::getState() const {
    return X_;
}

Eigen::Matrix<double, 9, 9> ExtendedKalmanFilter::getCovariance() const {
    return P_;
}

} // namespace dt46_vision
