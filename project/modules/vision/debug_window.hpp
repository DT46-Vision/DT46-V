#pragma once
#include "modules/vision/virtual_camera.hpp"
#include <memory>

namespace dt46 {
class VisionDebug {
public:
    VisionDebug(AimPipeline& pipeline,VirtualCamera& camera,std::filesystem::path output,bool visible=true);
    ~VisionDebug();
    bool pump();
    void show();
    void update(const VirtualFrame& frame,const Json& result);
    void save_frame();
private:
    struct Impl;std::unique_ptr<Impl> impl;
};
}
