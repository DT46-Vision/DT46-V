#include "modules/vision/virtual_camera.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <future>
#include <iostream>
#include <stdexcept>

namespace {
void camera_raw_frame_check() {
    using dt46::Json;using dt46::wire::Socket;
    std::uint16_t port=0;auto listener=Socket::listen(port);
    const Json frame_info={{"width",2},{"height",2},{"color_order","RGB"},
        {"K",{{100,0,.7},{0,105,.3},{0,0,1}}},{"D",{-.1,.02,.003,-.004,0}},
        {"optical",{{0,0,1,1},{-1,0,0,2},{0,-1,0,3},{0,0,0,1}}},{"muzzle_offset",{.03,-.04,.05}}};
    const std::vector<std::uint8_t> pixels={1,2,3,4,5,6,7,8,9,10,11,12};
    auto server=std::async(std::launch::async,[&]{
        auto socket=listener.accept();socket.receive();socket.send({{"protocol",1}});
        Json parameters={{"exposure_time",2500}};int version=0;
        try{while(true){const auto request=socket.receive().metadata;const auto op=request.at("op");
            if(op=="describe")socket.send({{"protocol",1},{"camera",parameters}});
            else if(op=="next")socket.send({{"version",++version},{"frame",frame_info}},pixels);
            else{
                if(request.at("parameters").contains("flip_image"))throw std::runtime_error("Removed flip parameter reached sensor service");
                if(op=="configure_camera")parameters.update(request.at("parameters"));
                socket.send({{"parameters",parameters}});
            }
        }}catch(const dt46::wire::Disconnected&){}
    });
    {
        dt46::VirtualCamera camera({{"port",port},{"key","test"}});
        if(camera.description.at("camera").contains("flip_image"))throw std::runtime_error("Removed flip control is still advertised");
        camera.configure({{"exposure_time",1250}});const auto configured=camera.description;
        if(configured.at("camera").at("exposure_time")!=1250)throw std::runtime_error("Camera exposure configuration failed");
        for(const Json& flip:Json::array({true,false,1,0})) {
            bool rejected=false;try{camera.configure({{"flip_image",flip},{"exposure_time",9999}});}catch(const std::invalid_argument&){rejected=true;}
            if(!rejected||camera.description!=configured)throw std::runtime_error("Legacy flip configuration was accepted or changed settings");
        }
        const auto raw=camera.next(Json::object());
        if(!raw||raw->metadata!=frame_info||!std::equal(pixels.begin(),pixels.end(),raw->rgb.data))
            throw std::runtime_error("Raw camera pixels or geometry metadata changed");
    }
    server.get();
}
}

int main(int argc, char** argv) {
    try {
        if (argc != 2) throw std::runtime_error("Expected repository root");
        dt46::AimPipeline pipeline(argv[1]);
        const auto* initial = pipeline.context();
        const auto revision = pipeline.revision();
        bool rejected = false;
        try { pipeline.configure({{"tracker", {{"gravity", 9.8}}}}); } catch (const std::exception&) { rejected = true; }
        if (!rejected || pipeline.context() != initial || pipeline.revision()!=revision) throw std::runtime_error("Original tracker gravity constraint ignored");
        rejected = false;
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
        pipeline.configure({{"detector",{{"binary_val",40},{"detect_color",0}}},{"tracker",{{"follow_decision",false},{"target_color",0}}}});
        cv::Mat red=image.clone();cv::rectangle(red,{26,10,5,36},{255,0,0},cv::FILLED);
        const auto manual=pipeline.process(red,0,k,{},cv::Matx33d::eye(),{},1);
        if(manual.at("input").at("target_color")!=0||manual.at("light_count")!=1)throw std::runtime_error("Manual target/red detection ignored");
        pipeline.configure({{"detector",{{"detect_color",1}}},{"tracker",{{"follow_decision",true}}}});
        const auto blue=pipeline.process(red,0,k,{},cv::Matx33d::eye(),{},1);
        if(blue.at("input").at("target_color")!=1||blue.at("light_count")!=0)throw std::runtime_error("Decision target/blue filter ignored");
        pipeline.configure({{"detector",{{"detect_color",2}}}});
        if(pipeline.process(red,0,k,{},cv::Matx33d::eye(),{},1).at("light_count")!=1)throw std::runtime_error("Both-color detection ignored");
        const auto* configured=pipeline.context();const auto configured_revision=pipeline.revision();
        for(const auto& invalid:std::vector<dt46::Json>{{{"model","missing-rqt-test.onnx"}},{{"detector",{{"detect_color",3}}}},{{"tracker",{{"text_size",0}}}},{{"tracker",{{"display",0.5}}}}}){
            rejected=false;try{pipeline.configure(invalid);}catch(const std::exception&){rejected=true;}
            if(!rejected||pipeline.context()!=configured||pipeline.revision()!=configured_revision)throw std::runtime_error("Invalid model/display/color replaced context");
        }
        camera_raw_frame_check();
        std::cout << "Native configuration, colors, raw camera geometry, retired flip rejection, monotonic timestamps and reset passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
