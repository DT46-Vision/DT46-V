#include "modules/vision/virtual_camera.hpp"
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    try {
        if (argc != 2) throw std::runtime_error("Expected repository root");
        dt46::AimPipeline pipeline(argv[1]);
        const auto* initial = pipeline.context();
        const auto revision = pipeline.revision();
        bool rejected = false;
        try { pipeline.configure({{"detector", {{"roi_scale", 0}}}}); } catch (const std::exception&) { rejected = true; }
        if (!rejected || pipeline.context() != initial || pipeline.revision()!=revision) throw std::runtime_error("Invalid configuration changed the pipeline");
        const cv::Mat image(64,64,CV_8UC3,cv::Scalar(0,0,0));
        const cv::Matx33d k(100,0,31.5,0,100,31.5,0,0,1);
        const auto result = pipeline.process(image, 0, k, {}, cv::Matx33d::eye(), {}, 1);
        if (!result.at("observations").empty() || result.at("state") != "LOST") throw std::runtime_error("Blank frame unexpectedly tracked");
        if(result.at("aim").at("valid").get<bool>()||result.at("aim").at("can_fire").get<bool>())throw std::runtime_error("Blank frame returned an aim command");
        auto tagged=result;tagged.update({{"frame_number",2},{"state_tick",30},{"reset_generation",4},{"camera","onboard"},{"robot_id","infantry3"}});
        const auto feedback=dt46::aim_feedback(tagged);
        if(feedback.at("aim")!=result.at("aim")||feedback.at("reset_generation")!=4||feedback.contains("observations")||!dt46::aim_feedback(nullptr).is_null())throw std::runtime_error("Invalid compact aim feedback");
        rejected = false;
        try { pipeline.process(image, 0, k, {}, cv::Matx33d::eye(), {}, 1); } catch (const std::exception&) { rejected = true; }
        if (!rejected) throw std::runtime_error("Repeated timestamp accepted");
        pipeline.reset();
        if(pipeline.revision()!=revision+1)throw std::runtime_error("Reset did not invalidate previous aim results");
        pipeline.process(image, 0, k, {}, cv::Matx33d::eye(), {}, 1);
        std::cout << "Native configuration, frame processing, monotonic timestamps and reset passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
