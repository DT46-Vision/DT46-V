#ifndef PNP_H
#define PNP_H

#include <opencv2/opencv.hpp>
#include "rm_detector/detector.hpp"
#include <array>
#include <cmath>
#include <vector>
#include <limits>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;

namespace DT46_VISION {

    struct CameraCalibration {
        std::array<double, 9> k;
        std::vector<double> d;
    };

    class PNP {
    public:
        PNP() = default;

        // 处理装甲板灯点坐标并返回 (dx, dy, dz, yaw_deg, nx, ny, nz)
        // nx, ny, nz 是相机系下指向板外的法线
        std::tuple<double, double, double, double, double, double, double> processArmorCorners(
        const CameraCalibration* cam_info,
        bool use_geometric_center, // 是否用图像几何中心替代标定光心
        const cv::Mat& frame,
        const Armor& armor,
        int class_id
        );

        // Clear the previous diagnostic pose at frame boundaries.
        void beginFrame();
        void setArmorGeometry(double light_spacing_mm,double light_height_mm);
        struct Pose {
            std::vector<cv::Point3d> corners_camera_mm;
            double reprojection_error_px = 0;
        };
        const Pose& pose() const { return pose_; }
        void resetCameraInfo() { has_cached_caminfo_=false; pose_={}; }

    private:
        cv::Mat cached_K_;
        cv::Mat cached_D_;
        bool has_cached_caminfo_ = false;
        double light_spacing_mm_=135.0,light_height_mm_=58.0;
        Pose pose_;

        void parseCameraInfo(const CameraCalibration* msg);
        std::vector<cv::Point3f> getObjectPoints(int object_size);
    };

}

#endif // PNP_H
