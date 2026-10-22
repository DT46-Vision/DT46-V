#pragma once
#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace dt46::hardware {
std::uint8_t crc8(std::span<const std::uint8_t> bytes,std::uint8_t initial=0xff);
std::uint16_t mcu_crc16(std::span<const std::uint8_t> bytes,std::uint16_t initial=0xffff);
std::uint16_t dm_crc16(std::span<const std::uint8_t> bytes);
struct GimbalCommand {float yaw=0,pitch=0;std::int32_t can_fire=0;};
struct McuFeedback {std::uint8_t color=0;float roll=0,pitch=0,yaw=0,speed=0;};
struct ImuSample {std::uint8_t rid=0;std::array<float,3> rpy{};};
std::array<std::uint8_t,15> encode(const GimbalCommand& command);
std::array<double,4> quaternion(const std::array<float,3>& degrees);
class McuParser {
public:
    std::vector<McuFeedback> feed(std::span<const std::uint8_t> bytes);
    void reset(){size=0;}
    std::uint64_t accepted=0,crc_failures=0;
private:
    std::array<std::uint8_t,20> buffer{};std::size_t size=0;
};
class ImuParser {
public:
    std::vector<ImuSample> feed(std::span<const std::uint8_t> bytes);
    void reset(){size=0;}
    std::uint64_t accepted=0,crc_failures=0,invalid_frames=0,discarded_bytes=0;
private:
    std::array<std::uint8_t,19> buffer{};std::size_t size=0;
};
}
