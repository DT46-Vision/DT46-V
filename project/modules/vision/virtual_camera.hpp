#pragma once
#include "interfaces/transport.hpp"
#include "modules/vision/pipeline.hpp"
#include <optional>
#include <memory>

namespace dt46 {
struct VirtualFrame {cv::Mat rgb;Json metadata;std::shared_ptr<std::vector<std::uint8_t>> storage;};
class VirtualCamera {
public:
    explicit VirtualCamera(const Json& session);
    Json request(const Json& request);
    std::optional<VirtualFrame> next(const Json& status,const Json& result=nullptr);
    void configure(const Json& parameters);
    void validate(const Json& parameters);
    Json description;
private:
    wire::Socket socket;std::uint64_t version=0;
};
Json process_frame(AimPipeline& pipeline,const VirtualFrame& frame);
Json aim_feedback(const Json& result);
}
