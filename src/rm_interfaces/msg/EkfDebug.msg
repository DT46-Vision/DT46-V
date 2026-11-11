# EKF 调试数据，供 PlotJuggler 观察滤波器内部量。
# 状态顺序统一为 [x, vx, y, vy, z, vz, yaw, vyaw, r]，世界系，单位 m / m/s / rad。
std_msgs/Header header

# ---------- 时间 ----------
bool    valid                  # EKF 是否已初始化
float64 dt                     # 本次 predict 使用的帧间隔 (s)

# ---------- 1. 观测 / 预测 / 滤波 ----------
# 观测：当前帧喂给 EKF 的装甲板（世界系）
float64 meas_x
float64 meas_y
float64 meas_z
float64 meas_yaw

# 预测：先验车心推出的装甲板位置，用于和新息对照
float64 pred_x
float64 pred_y
float64 pred_z
float64 pred_yaw

# 先验车心状态（predict 后、update 前）
float64 prior_x
float64 prior_vx
float64 prior_y
float64 prior_vy
float64 prior_z
float64 prior_vz
float64 prior_yaw
float64 prior_vyaw
float64 prior_r

# 后验车心状态（Tracker 最终实际使用值，含限幅/小陀螺修正）
float64 x
float64 vx
float64 y
float64 vy
float64 z
float64 vz
float64 yaw
float64 vyaw
float64 r
float64 another_r              # 另一对装甲板半径
float64 dz                     # 高低板高度差

# ---------- 2. 残差 / 新息 ----------
# 先验新息 Y = z - h(x_prior)
float64 innov_x
float64 innov_y
float64 innov_z
float64 innov_yaw

# 后验残差 = z - h(x_post)
float64 postfit_x
float64 postfit_y
float64 postfit_z
float64 postfit_yaw

# 白化新息（单位方差，便于按轴定位异常）
float64 nis_x
float64 nis_y
float64 nis_z
float64 nis_yaw

# 新息相对视线的径向/切向分量（切向大 => 换板/角度问题，径向大 => 深度问题）
float64 innov_radial
float64 innov_tangential

# ---------- 3. 误差协方差对角线 ----------
# 后验 P
float64 p_xx
float64 p_vx
float64 p_yy
float64 p_vy
float64 p_zz
float64 p_vz
float64 p_yaw
float64 p_vyaw
float64 p_r

# 先验 P（predict 后）
float64 p_prior_xx
float64 p_prior_vx
float64 p_prior_yy
float64 p_prior_vy
float64 p_prior_zz
float64 p_prior_vz
float64 p_prior_yaw
float64 p_prior_vyaw
float64 p_prior_r

# 新息协方差 S 对角线
float64 s_x
float64 s_y
float64 s_z
float64 s_yaw

# 观测噪声 R 对角线（随距离动态变化）
float64 r_x
float64 r_y
float64 r_z
float64 r_yaw

# 过程噪声 Q 对角线
float64 q_x
float64 q_vx
float64 q_y
float64 q_vy
float64 q_z
float64 q_vz
float64 q_yaw
float64 q_vyaw
float64 q_r

# ---------- 4. 统计检验 ----------
float64 mahalanobis_sq         # 马氏距离平方（卡方检验量）
float64 mahalanobis_threshold  # 拒收阈值，当前代码为 30.0
bool    has_measurement        # 本帧是否送入装甲板观测
bool    accepted               # 本帧观测是否被接受
bool    reset                  # 本帧是否发生 EKF 重开
int32   reject_streak          # 连续被拒次数

# ---------- 车心伪观测（SPIN 双板/射线软融合） ----------
bool    center_has_measurement
bool    center_accepted
float64 center_meas_x
float64 center_meas_y
float64 center_sigma           # 伪观测标准差
float64 center_innov_x
float64 center_innov_y
float64 center_mahalanobis_sq

# ---------- 跟踪状态标志 ----------
int32   tracker_state          # 0 LOST / 1 DETECTING / 2 TRACKING / 3 TEMP_LOST
bool    spin
bool    radius_railed          # 半径是否被限幅（触发可能的重开）
int32   armor_id
