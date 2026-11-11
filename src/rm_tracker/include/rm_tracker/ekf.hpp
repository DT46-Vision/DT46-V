#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace dt46_vision {

// EKF 内部调试快照。状态顺序 [x, vx, y, vy, z, vz, yaw, vyaw, r]。
struct EkfDebugData {
    bool valid = false;
    bool has_measurement = false;
    bool accepted = false;
    double dt = 0.0;

    Eigen::Vector4d measurement = Eigen::Vector4d::Zero();
    Eigen::Vector4d predicted_armor = Eigen::Vector4d::Zero();

    Eigen::Matrix<double, 9, 1> prior_state = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 1> posterior_state = Eigen::Matrix<double, 9, 1>::Zero();

    Eigen::Matrix<double, 9, 1> P_diag = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 1> P_prior_diag = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 1> Q_diag = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 4, 1> R_diag = Eigen::Matrix<double, 4, 1>::Zero();
    Eigen::Matrix<double, 4, 1> S_diag = Eigen::Matrix<double, 4, 1>::Zero();

    Eigen::Vector4d innovation = Eigen::Vector4d::Zero();
    Eigen::Vector4d postfit = Eigen::Vector4d::Zero();
    Eigen::Vector4d nis = Eigen::Vector4d::Zero();
    double innov_radial = 0.0;
    double innov_tangential = 0.0;

    double mahalanobis_sq = 0.0;
    double mahalanobis_threshold = 30.0;

    bool center_has_measurement = false;
    bool center_accepted = false;
    Eigen::Vector2d center_measurement = Eigen::Vector2d::Zero();
    Eigen::Vector2d center_innovation = Eigen::Vector2d::Zero();
    double center_sigma = 0.0;
    double center_mahalanobis_sq = 0.0;
};

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
     * @brief 更新步。测量是当前这块装甲板 [xa, ya, za, yaw]
     * @param measurement 世界系下装甲位置和连续 yaw
     * @return false 表示马氏距离过大，这帧观测被拒绝
     */
    bool update(const Eigen::Vector4d& measurement);

    /**
     * @brief 车心伪观测更新：射线/双板解出的 (xc,yc) 软融合进状态，不硬盖。
     * @param sigma_m 观测标准差 (m)，双板可更小、单板射线更大
     */
    bool update_center_xy(double xc_meas, double yc_meas, double sigma_m);

    /**
     * @brief 软重置协方差 (应对装甲板跳变)
     */
    void smooth_reset_covariance();

    // 状态读写接口
    void setState(const Eigen::Matrix<double, 9, 1>& state);
    Eigen::Matrix<double, 9, 1> getState() const;
    Eigen::Matrix<double, 9, 9> getCovariance() const;
    const EkfDebugData& getDebug() const { return debug_; }

private:
    bool is_initialized_;

    Eigen::Matrix<double, 9, 1> X_; // 状态向量
    Eigen::Matrix<double, 9, 9> P_; // 状态协方差矩阵
    Eigen::Matrix<double, 9, 9> F_; // 状态转移矩阵
    Eigen::Matrix<double, 9, 9> Q_; // 过程噪声矩阵
    Eigen::Matrix<double, 4, 4> R_; // 观测噪声矩阵

    EkfDebugData debug_;

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
