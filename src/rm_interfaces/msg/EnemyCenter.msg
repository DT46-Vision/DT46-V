# 敌人车辆中心信息（视觉 → 导航）
std_msgs/Header header   # 时间戳 = 图像曝光时刻；frame_id = tracking_frame
float32 yaw              # 云台需要转动的 yaw 增量（度），与 /tracker/gimbal_control 的 yaw 同值
float32 x                # 敌人车辆中心 x，世界惯性系，米（EKF 状态[0]）
float32 y                # 敌人车辆中心 y，世界惯性系，米（EKF 状态[2]）
float32 z                # 敌人车辆中心 z，世界惯性系，米（EKF 状态[4]）
float32 enemy_yaw        # 敌人车体朝向，世界系，弧度（EKF 状态[6]）
int32   armor_id         # 兵种编号 0-11（B1-B7/R1-R7，表见 README）
bool    tracked          # 有效跟踪标志（非 LOST 为 true）
