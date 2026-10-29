#include "rm_detector/pnp.hpp"
#include "rm_tracker/rm_tf_tools.hpp"
#include <iostream>
#include <stdexcept>

int main() {
    DT46_VISION::CameraCalibration calibration{{2100, 0, 612, 0, 2080, 493, 0, 0, 1},
                                               {-0.1, 0.02, 0.001, -0.002, 0.0}};
    cv::Mat k(3, 3, CV_64F, calibration.k.data());
    cv::Mat d(calibration.d);
    const std::vector<cv::Point3f> object{{-67.5f,-29,0}, {-67.5f,29,0},
                                        {67.5f,-29,0}, {67.5f,29,0}};
    const cv::Vec3d position(50, -20, 2300);
    std::vector<cv::Point2f> image;
    cv::projectPoints(object, cv::Vec3d(0.05, -0.12, 0.02), position, k, d, image);
    DT46_VISION::Light left(image[0], image[1], 1), right(image[2], image[3], 1);
    DT46_VISION::Armor armor(2.0, left, right, {2, 1.0f}, false, {});
    armor.light1_up=image[0]; armor.light1_down=image[1];
    armor.light2_up=image[2]; armor.light2_down=image[3];
    DT46_VISION::PNP pnp;
    pnp.beginFrame();
    auto [x,y,z,yaw,nx,ny,nz] = pnp.processArmorCorners(
        &calibration, false, cv::Mat::zeros(1024,1280,CV_8UC3), armor, 5);
    const double error = cv::norm(cv::Vec3d(x,y,z)-position);
    if (!std::isfinite(error) || error > 2.0)
        throw std::runtime_error("PnP failed with source K/D and an off-centre principal point");
    const std::vector<cv::Point3f> measured_geometry{{-62.9f,-29.735f,0},{-62.9f,29.735f,0},{62.9f,-29.735f,0},{62.9f,29.735f,0}};
    cv::projectPoints(measured_geometry,cv::Vec3d(.15,-.4,.02),position,k,d,image);
    armor.light1_up=image[0];armor.light1_down=image[1];armor.light2_up=image[2];armor.light2_down=image[3];
    pnp.setArmorGeometry(125.8,59.47);pnp.resetCameraInfo();pnp.beginFrame();
    const auto measured=pnp.processArmorCorners(&calibration,false,cv::Mat::zeros(1,1,CV_8UC3),armor,5);
    if(cv::norm(cv::Vec3d(std::get<0>(measured),std::get<1>(measured),std::get<2>(measured))-position)>2)
        throw std::runtime_error("Configured light geometry did not reach actual PnP");
    bool invalid_geometry=false;try{pnp.setArmorGeometry(0,58);}catch(const std::invalid_argument&){invalid_geometry=true;}
    if(!invalid_geometry)throw std::runtime_error("Invalid armor geometry accepted");
    DT46_VISION::CameraCalibration source{{1075.77876,0,649.75819,0,1075.95573,525.30871,0,0,1},
                                         {-0.454786,0.179046,-0.001430,-0.000463,0}};
    cv::Mat source_k(3,3,CV_64F,source.k.data());
    for(const auto& target:std::vector<cv::Vec3d>{{0,0,2300},{800,300,2300},{-800,-300,2300},{1100,-550,2300}}) {
        cv::projectPoints(object,cv::Vec3d(.05,-.12,.02),target,source_k,cv::Mat(source.d),image);
        armor.light1_up=image[0];armor.light1_down=image[1];armor.light2_up=image[2];armor.light2_down=image[3];
        double errors[2];
        for(int ignore=0;ignore<2;++ignore) {
            auto current=source;if(ignore)current.d.assign(5,0);
            DT46_VISION::PNP solver;solver.beginFrame();
            const auto [px,py,pz,ry,ax,ay,az]=solver.processArmorCorners(&current,false,cv::Mat::zeros(1024,1280,CV_8UC3),armor,5);
            errors[ignore]=cv::norm(cv::Vec3d(px,py,pz)-target);
        }
        std::cout<<"distortion_check target_mm="<<target<<" compensated_mm="<<errors[0]<<" ignored_mm="<<errors[1]<<std::endl;
        if(!std::isfinite(errors[0])||(std::abs(target[0])>700&&errors[0]>=errors[1]*.1))
            throw std::runtime_error("Original PnP distortion compensation failed the centre/periphery control");
    }
    dt46_vision::RmTF tf;
    tf.set_camera_info(k, d, 1280, 1024, false);
    if (std::get<0>(tf.project_point(Eigen::Vector3d(0,0,2))) != cv::Point2i(612,493))
        throw std::runtime_error("Tracker projection replaced the calibrated principal point");
    dt46_vision::RmTF legacy;
    legacy.set_camera_info(k, d, 1280, 1024);
    if (std::get<0>(legacy.project_point(Eigen::Vector3d(0,0,2))) != cv::Point2i(640,512))
        throw std::runtime_error("Default ROS projection changed");
    const Eigen::Vector3d point(0.03, -0.04, 2.0);
    for (double angle : {-0.7, 0.0, 0.5}) {
        const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitX())).toRotationMatrix();
        tf.set_optical_rotation(rotation);
        const Eigen::Vector3d outward(-std::cos(.7)*std::cos(.26),-std::sin(.7)*std::cos(.26),std::sin(.26));
        if(std::abs(tf.armor_yaw_from_normal(rotation.transpose()*outward,Eigen::Vector3d::Zero(),0)-.7)>1e-12 ||
           tf.armor_yaw_from_normal(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),.12)!=.12)
            throw std::runtime_error("Inclined armor normal lost during optical conversion");
        const auto world = tf.cam_to_world(point, Eigen::Vector3d::Zero());
        if ((world-rotation*point).norm() > 1e-12 ||
            (tf.world_to_cam(world, Eigen::Vector3d::Zero())-point).norm() > 1e-12)
            throw std::runtime_error("Optical transform round trip failed");
    }
    std::cout << "PASS: calibrated PnP error " << error << " mm; optical rotation round trips\n";
}
