#include "modules/vision/virtual_camera.hpp"
#include <chrono>
#include <cstring>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace dt46 {
namespace {
double camera_wall_seconds() {
#ifdef _WIN32
    static const double frequency=[] {LARGE_INTEGER value;QueryPerformanceFrequency(&value);return double(value.QuadPart);}();
    LARGE_INTEGER value;QueryPerformanceCounter(&value);return double(value.QuadPart)/frequency;
#else
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
}
}
VirtualCamera::VirtualCamera(const Json& session):socket(wire::Socket::connect(session.at("port"))) {
    socket.send({{"key",session.at("key")}});const auto hello=socket.receive();
    if(hello.metadata.value("protocol",0)!=1)throw std::runtime_error("Virtual camera authentication/protocol failed");
    description=request({{"op","describe"}});if(description.at("protocol")!=1)throw std::runtime_error("Unsupported virtual camera protocol");
}
Json VirtualCamera::request(const Json& request) {
    socket.send(request);const auto reply=socket.receive();
    if(reply.metadata.contains("error"))throw std::runtime_error(reply.metadata.at("error").get<std::string>());return reply.metadata;
}
std::optional<VirtualFrame> VirtualCamera::next(const Json& status,const Json& result) {
    socket.send({{"op","next"},{"after",version},{"status",status},{"result",result}});auto reply=socket.receive();
    const double received=camera_wall_seconds();
    if(reply.metadata.contains("error"))throw std::runtime_error(reply.metadata.at("error").get<std::string>());
    version=reply.metadata.at("version");auto frame=reply.metadata.at("frame");if(frame.is_null()){if(!reply.bytes.empty())throw std::runtime_error("Unexpected camera payload");return {};}
    const int width=frame.at("width"),height=frame.at("height");
    if(width<=0||height<=0||width>8192||height>8192||frame.at("color_order")!="RGB"||reply.bytes.size()!=std::size_t(width)*height*3)
        throw std::runtime_error("Invalid virtual camera RGB shape");
    auto storage=std::make_shared<std::vector<std::uint8_t>>(std::move(reply.bytes));
    cv::Mat rgb(height,width,CV_8UC3,storage->data());
    if(frame.contains("camera_timing")) {
        auto& timing=frame["camera_timing"];timing["receive_complete_s"]=received;
        const double available=camera_wall_seconds();timing["image_available_s"]=available;
        frame["camera_ms"]=(available-timing.at("sample_s").get<double>())*1000;
    }
    return VirtualFrame{std::move(rgb),std::move(frame),std::move(storage)};
}
void VirtualCamera::configure(const Json& parameters){
    validate(parameters);
    description["camera"]=request({{"op","configure_camera"},{"parameters",parameters}}).at("parameters");
}
void VirtualCamera::validate(const Json& parameters){
    if(parameters.contains("flip_image"))throw std::invalid_argument("flip_image is no longer supported; remove it from camera settings and use display rotation");
    request({{"op","validate_camera"},{"parameters",parameters}});
}
Json aim_feedback(const Json& result) {
    if(result.is_null())return nullptr;
    Json feedback={{"schema",1}};
    for(const auto* key:{"frame_number","state_tick","reset_generation","simulation_time_s","camera","robot_id","state","tracked_id","aim"})feedback[key]=result.at(key);
    return feedback;
}
Json process_frame(AimPipeline& pipeline,const VirtualFrame& frame) {
    const auto start=std::chrono::steady_clock::now();const auto& m=frame.metadata;
    cv::Matx33d k,rotation;cv::Vec<double,5> distortion;cv::Vec3d offset;
    for(int row=0;row<3;++row)for(int col=0;col<3;++col){k(row,col)=m.at("K").at(row).at(col);rotation(row,col)=m.at("optical").at(row).at(col);}
    for(int i=0;i<5;++i)distortion[i]=m.at("D").at(i);for(int i=0;i<3;++i)offset[i]=m.at("muzzle_offset").at(i);
    const double speed=m.at("speed");auto result=pipeline.process(frame.rgb,m.at("time"),k,distortion,rotation,offset,m.at("color"),speed);
    result.update({{"camera",m.at("camera")},{"robot_id",m.at("robot_id")},{"detect_color",result.at("input").at("detect_color")},{"reset_generation",m.at("generation")},
        {"transport","virtual_camera"},{"bullet_speed_m_s",speed?speed:pipeline.nominal_speed()},{"bullet_speed_source",speed?"dual_photogate":"tracker_config"},
        {"camera_ms",m.value("camera_ms",0.)},{"processing_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()},
        {"frame_number",m.at("frame_number")},{"state_tick",m.at("state_tick")},{"sensor",m.at("sensor")},{"parameter_revision",pipeline.revision()}});
    if(m.contains("appearance"))result["appearance"]=m.at("appearance");
    if(m.contains("camera_timing"))result["camera_timing"]=m.at("camera_timing");
    for(const char* key:{"camera_settings_version","sampling_target_hz","sampling_effective_limit_hz"})if(m.contains(key))result[key]=m.at(key);
    result["input"]["optical_transform"]=m.at("optical");return result;
}
}
