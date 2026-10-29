#include "rm_detector/pnp.hpp"
#include <stdexcept>

namespace DT46_VISION {
    void PNP::setArmorGeometry(double spacing,double height) {
        if(!std::isfinite(spacing+height)||spacing<=0||height<=0)throw std::invalid_argument("Armor light geometry must be positive finite millimeters");
        light_spacing_mm_=spacing;light_height_mm_=height;prev_plates_.clear();curr_plates_.clear();
    }

    void PNP::beginFrame() {
        prev_plates_ = std::move(curr_plates_);
        curr_plates_.clear();
    }

    void PNP::parseCameraInfo(const CameraCalibration* msg) {
        if (msg == nullptr || has_cached_caminfo_) return;

        cached_K_ = (cv::Mat_<double>(3, 3) <<
            msg->k[0], msg->k[1], msg->k[2],
            msg->k[3], msg->k[4], msg->k[5],
            msg->k[6], msg->k[7], msg->k[8]);

        cached_D_ = cv::Mat(msg->d).reshape(1, 1).clone();

        has_cached_caminfo_ = true;
    }

    // 根据物体尺寸ID选择物体3D坐标 (单位: mm)
    std::vector<cv::Point3f> PNP::getObjectPoints(int object_size) {
        // 定义灯条和装甲板的物理尺寸
        // 坐标系定义: X右, Y下, Z前 (物体平面)
        // 确保这里的顺序与 imagePoints (左上, 左下, 右上, 右下) 一一对应
        (void)object_size;
        const float half_width=float(light_spacing_mm_*.5),half_height=float(light_height_mm_*.5);
        return {
            {-half_width, -half_height, 0.0f},  // 0: 左上
            {-half_width,  half_height, 0.0f},  // 1: 左下
            { half_width, -half_height, 0.0f},  // 2: 右上
            { half_width,  half_height, 0.0f}   // 3: 右下
        };
    }

    // 处理装甲板灯点坐标并返回 (dx, dy, dz)
    std::tuple<double, double, double, double, double, double, double> PNP::processArmorCorners(
        const CameraCalibration* cam_info,
        bool use_geometric_center, // 是否用图像几何中心替代标定光心
        const cv::Mat& frame,
        const Armor& armor,
        int class_id
        ) {
        // 提取四个灯条端点
        std::vector<cv::Point2f> imagePoints = {
            armor.light1_up,
            armor.light1_down,
            armor.light2_up,
            armor.light2_down
        };

        if (imagePoints.size() != 4) {
            std::cerr << "Invalid armor corner count: " << imagePoints.size() << '\n';
            return {0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0};
        }

        // 获取相机参数
        cv::Mat cameraMatrix, distCoeffs;
        parseCameraInfo(cam_info);

        if (!has_cached_caminfo_) {
            return {0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0};
        }

        if (has_cached_caminfo_) {
            cameraMatrix = cached_K_;
            distCoeffs = cached_D_;
        }

        // 强制使用图像几何中心 (仅在特定裁切策略下使用)
        if (use_geometric_center) {
            if (!has_cached_caminfo_) {
                cameraMatrix = cv::Mat_<double>(3, 3);
            }
            cameraMatrix.at<double>(0,2) = frame.cols / 2.0;
            cameraMatrix.at<double>(1,2) = frame.rows / 2.0;
        }

        std::vector<cv::Point3f> objectPoints = getObjectPoints(class_id);

        // 3. 平面目标 PnP 有两个解（近/远），必须显式消歧：
        //    用“尺寸估计深度”和上一帧深度做选择，避免深度在 2m/4m 两个模式之间跳。
        std::vector<cv::Mat> rvecs, tvecs;
        int nsol = cv::solvePnPGeneric(objectPoints, imagePoints,
                                       cameraMatrix, distCoeffs,
                                       rvecs, tvecs,
                                       false, cv::SOLVEPNP_IPPE);

        if (nsol <= 0 || rvecs.empty() || tvecs.empty()) {
            std::cerr << "solvePnPGeneric failed\n";
            return {0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0};
        }

        // 两个独立尺寸先验：
        //  - 宽度：左右灯条水平间距，正对时准，斜视(yaw)时被 cos(yaw) 压缩 → z_width 偏大
        //  - 高度：灯条竖直长度，对 yaw 不敏感（只受 pitch 影响）→ 斜视时更接近真值
        auto px_dist = [](const cv::Point2f& a, const cv::Point2f& b) {
            return std::hypot(a.x - b.x, a.y - b.y);
        };
        const double fx = cameraMatrix.at<double>(0, 0);
        const double fy = cameraMatrix.at<double>(1, 1);
        const double armor_width_mm  = light_spacing_mm_;
        const double armor_height_mm = light_height_mm_;
        const double w_px = 0.5 * (px_dist(imagePoints[2], imagePoints[0]) +
                                   px_dist(imagePoints[3], imagePoints[1]));
        const double h_px = 0.5 * (px_dist(imagePoints[0], imagePoints[1]) +
                                   px_dist(imagePoints[2], imagePoints[3]));
        const double z_width  = (w_px > 1.0) ? fx * armor_width_mm  / w_px  : 0.0;
        const double z_height = (h_px > 1.0) ? fy * armor_height_mm / h_px : 0.0;
        // 两个先验都偏大或接近真值，取较小者作为深度先验，避免斜视时把远解选进来
        double z_cue = 0.0;
        if (z_width > 0.0 && z_height > 0.0) z_cue = std::min(z_width, z_height);
        else z_cue = std::max(z_width, z_height);

        // 同一数字的两块板在图像上是分开的；只有图像位置接近的才是上一帧那一块
        const cv::Point2f img_center =
            (imagePoints[0] + imagePoints[1] + imagePoints[2] + imagePoints[3]) * 0.25f;
        const PlatePrior* prev = nullptr;
        double best_img = 120.0;
        for (const auto& plate : prev_plates_) {
            const double d = std::hypot(plate.cx - img_center.x, plate.cy - img_center.y);
            if (d < best_img) {
                best_img = d;
                prev = &plate;
            }
        }
        const bool has_prev = prev != nullptr;

        int best = 0;
        double best_score = std::numeric_limits<double>::max();
        for (int i = 0; i < nsol; ++i) {
            const double z = tvecs[i].at<double>(2);
            cv::Mat R_i;
            cv::Rodrigues(rvecs[i], R_i);
            const cv::Vec3d n(R_i.at<double>(0, 2), R_i.at<double>(1, 2), R_i.at<double>(2, 2));

            double score = 0.0;
            if (z <= 0.0) score += 1e6;  // 深度必须在相机前方

            if (has_prev) {
                // 实测两个 IPPE 解的水平 yaw 很接近，差别在法线的竖直分量上来回翻，只比 yaw 锁不住。
                // 用整条法线的夹角；尺寸先验继续压远解。单位 1rad 对 1m。
                const double dot = std::max(-1.0, std::min(1.0, n[0] * prev->nx + n[1] * prev->ny + n[2] * prev->nz));
                const double dang = std::acos(dot);
                const double dz = std::abs(z - prev->depth);
                score += dang + dz / 1000.0;
                if (z_cue > 0.0) score += std::abs(z - z_cue) / 1000.0;
                if (dz > 1000.0) score += 1e3;
            } else {
                // 初始化：装甲板法线应朝向相机（与 -tvec 方向夹角小），尺寸先验只做 tie-break
                cv::Vec3d dir(-tvecs[i].at<double>(0), -tvecs[i].at<double>(1), -tvecs[i].at<double>(2));
                const double dn = cv::norm(dir);
                if (dn > 1e-9) dir /= dn;
                const double dot = n[0] * dir[0] + n[1] * dir[1] + n[2] * dir[2];
                const double ang = std::acos(std::max(-1.0, std::min(1.0, dot)));
                score += ang + 0.001 * ((z_cue > 0.0) ? std::abs(z - z_cue) : 0.0);
            }
            if (score < best_score) {
                best_score = score;
                best = i;
            }
        }

        cv::Mat rvec = rvecs[best].clone();
        cv::Mat tvec = tvecs[best].clone();
        cv::Mat R_best;
        cv::Rodrigues(rvec, R_best);
        curr_plates_.push_back(PlatePrior{
            tvec.at<double>(2),
            R_best.at<double>(0, 2), R_best.at<double>(1, 2), R_best.at<double>(2, 2),
            img_center.x, img_center.y});

        double dx = tvec.at<double>(0);
        double dy = tvec.at<double>(1);
        double dz = tvec.at<double>(2);

        // 4. 计算绝对 Yaw
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        double r13 = R.at<double>(0, 2);
        double r33 = R.at<double>(2, 2);
        double pnp_yaw_rad = atan2(r13, r33); // 此时是绝对角度

        // 归一化逻辑 (正对时 yaw 为 0)
        double normalized_yaw = pnp_yaw_rad + CV_PI;

        // 约束到 [-PI, PI]
        if (normalized_yaw > CV_PI) normalized_yaw -= 2 * CV_PI;
        if (normalized_yaw < -CV_PI) normalized_yaw += 2 * CV_PI;
        // -------------------------------------------------------------

        double yaw_deg = normalized_yaw * 180.0 / CV_PI;

        // 物体 +Z 在相机系下指向板外，跟踪器用同一套旋转把它变到世界系
        double nx = R_best.at<double>(0, 2);
        double ny = R_best.at<double>(1, 2);
        double nz = R_best.at<double>(2, 2);

        return {dx, dy, dz, yaw_deg, nx, ny, nz};
    }
}
