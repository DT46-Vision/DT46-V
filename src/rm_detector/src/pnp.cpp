#include "rm_detector/pnp.hpp"
#include <stdexcept>

namespace DT46_VISION {
    void PNP::setArmorGeometry(double spacing,double height) {
        if(!std::isfinite(spacing+height)||spacing<=0||height<=0)throw std::invalid_argument("Armor light geometry must be positive finite millimeters");
        light_spacing_mm_=spacing;light_height_mm_=height;pose_={};
    }

    void PNP::beginFrame() {
        pose_ = {};
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
        // Detector's historical "up" is the lower endpoint of the OpenCV rectangle.
        // Keep that winding: object X points right, Y up and +Z out towards the camera.
        (void)object_size;
        const float half_width=float(light_spacing_mm_*.5),half_height=float(light_height_mm_*.5);
        return {
            {-half_width, -half_height, 0.0f},  // left bottom
            {-half_width,  half_height, 0.0f},  // left top
            { half_width, -half_height, 0.0f},  // right bottom
            { half_width,  half_height, 0.0f}   // right top
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
        pose_ = {};
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
            cameraMatrix = cached_K_.clone();
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

        // 3. IPPE returns two planar pose candidates.
        //    Reprojection distinguishes them when the image has enough perspective information.
        std::vector<cv::Mat> rvecs, tvecs;
        int nsol = cv::solvePnPGeneric(objectPoints, imagePoints,
                                       cameraMatrix, distCoeffs,
                                       rvecs, tvecs,
                                       false, cv::SOLVEPNP_IPPE);

        if (nsol <= 0 || rvecs.empty() || tvecs.empty()) {
            std::cerr << "solvePnPGeneric failed\n";
            return {0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0};
        }

        // A temporal normal prior must not override the current image evidence.
        // Both IPPE candidates have similar depth; score their actual four-point residual.
        int best = -1;
        double best_score = std::numeric_limits<double>::max();
        for (int i = 0; i < nsol; ++i) {
            cv::Mat rotation;
            cv::Rodrigues(rvecs[i], rotation);
            bool in_front = true;
            for (const auto& point : objectPoints) {
                const cv::Mat vertex = rotation * (cv::Mat_<double>(3,1) << point.x, point.y, point.z) + tvecs[i];
                in_front = in_front && vertex.at<double>(2) > 0 && cv::checkRange(vertex);
            }
            if (!in_front) continue;
            std::vector<cv::Point2f> projected;
            cv::projectPoints(objectPoints, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, projected);
            double score = 0;
            for (int j = 0; j < 4; ++j) {
                const auto residual = projected[j] - imagePoints[j];
                score += residual.dot(residual);
            }
            if (std::isfinite(score) && score < best_score) { best_score = score; best = i; }
        }
        if (best < 0) return {0, 0, -1, 0, 0, 0, 0};
        const cv::Mat rvec = rvecs[best], tvec = tvecs[best];
        cv::Mat R_best;
        cv::Rodrigues(rvec, R_best);
        pose_.reprojection_error_px = std::sqrt(best_score / 4);
        for (const auto& point : objectPoints) {
            const cv::Mat vertex = R_best * (cv::Mat_<double>(3,1) << point.x, point.y, point.z) + tvec;
            pose_.corners_camera_mm.emplace_back(vertex.at<double>(0), vertex.at<double>(1), vertex.at<double>(2));
        }
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
