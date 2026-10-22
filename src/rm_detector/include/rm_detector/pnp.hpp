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

        // 每帧开始时调用：上一帧的板才参与消歧，同一帧的另一块板不能互相污染
        void beginFrame();

    private:
        struct PlatePrior {
            double depth;
            double nx;
            double ny;
            double nz;
            float cx;
            float cy;
        };

        cv::Mat cached_K_;
        cv::Mat cached_D_;
        bool has_cached_caminfo_ = false;
        // 按图像位置区分同一数字的多块板，只和上一帧、同一块板做时序消歧
        std::vector<PlatePrior> prev_plates_;
        std::vector<PlatePrior> curr_plates_;

        void parseCameraInfo(const CameraCalibration* msg);
        std::vector<cv::Point3f> getObjectPoints(int object_size);
    };

}

#endif // PNP_H
