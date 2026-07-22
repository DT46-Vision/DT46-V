#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace dt46_vision {

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();

    /**
      @brief 初始化 EKF 状态
     * @param init_x 初始状态向量 [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r]
     * @param init_p 初始状态协方差矩阵
     */
    void init(const Eigen::Matrix<double, 9, 1>& init_x, const Eigen::Matrix<double, 9, 9>& init_p);

    /**
     * @brief 初始化动态 Q 和 R 的基准参数
     */
    void init_QR(double q_xyz, double q_yaw, double q_r, double r_xyz_factor, double r_yaw, double stable_dist);

    /**
     * @brief 预测步 (Predict)
     * @param dt 两帧之间的时间差
     */
    void predict(double dt);

    /**
     * @brief 更新步 (Update) - 基于 4 维装甲板观测 [x, y, z, yaw]
     * @param measurement 装甲板在世界系下的 4 维观测值
     */
    void update(const Eigen::Vector4d& measurement);

    /**
     * @brief 软重置协方差 (应对装甲板跳变)
     */
    void smooth_reset_covariance();

    // 状态读写接口
    void setState(const Eigen::Matrix<double, 9, 1>& state);
    Eigen::Matrix<double, 9, 1> getState() const;
    Eigen::Matrix<double, 9, 9> getCovariance() const;

private:
    bool is_initialized_;

    Eigen::Matrix<double, 9, 1> X_; // 状态向量
    Eigen::Matrix<double, 9, 9> P_; // 状态协方差矩阵
    Eigen::Matrix<double, 9, 9> F_; // 状态转移矩阵
    Eigen::Matrix<double, 9, 9> Q_; // 过程噪声矩阵
    Eigen::Matrix<double, 4, 4> R_; // 观测噪声矩阵

    // 动态噪声计算参数
    double s2qxyz_, s2qyaw_, s2qr_;
    double r_xyz_factor_, r_yaw_, stable_dist_;

    // 工具函数：角度归一化 [-PI, PI]
    static inline double normalizeAngle(double angle) {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0) {
            a += 2.0 * M_PI;
        }
        return a - M_PI;
    }
};

} // namespace dt46_vision
