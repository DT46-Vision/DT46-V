#pragma once
#include <cmath>
#include <optional>

namespace dt46_vision {
struct FrameStep { bool accepted, reset; double dt; };
class FrameClock {
public:
    FrameStep advance(double stamp) {
        if (!std::isfinite(stamp) || (last_ && stamp<=*last_)) return {false,false,0};
        const double dt=last_ ? stamp-*last_ : 0;
        const bool reset=!last_ || dt>.5;
        last_=stamp;
        return {true,reset,reset?0:dt};
    }
private:
    std::optional<double> last_;
};
}
