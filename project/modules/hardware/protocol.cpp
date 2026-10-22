#include "modules/hardware/protocol.hpp"
#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <numbers>

namespace dt46::hardware {
namespace {
static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559);
constexpr auto dm_table=[] {
    std::array<std::uint16_t,256> table{};
    for(unsigned i=0;i<256;++i){std::uint16_t value=std::uint16_t(i<<8);for(int bit=0;bit<8;++bit)value=std::uint16_t((value<<1)^((value&0x8000)?0x1021:0));table[i]=value;}
    return table;
}();
std::uint16_t little16(const std::uint8_t* p){return std::uint16_t(p[0])|(std::uint16_t(p[1])<<8);}
float little_float(const std::uint8_t* p){std::uint32_t bits=0;for(int i=0;i<4;++i)bits|=std::uint32_t(p[i])<<(8*i);return std::bit_cast<float>(bits);}
void put32(std::uint8_t* p,std::uint32_t bits){for(int i=0;i<4;++i)p[i]=std::uint8_t(bits>>(8*i));}
template<std::size_t N> void discard(std::array<std::uint8_t,N>& buffer,std::size_t& size,std::size_t count=1){std::move(buffer.begin()+count,buffer.begin()+size,buffer.begin());size-=count;}
}
std::uint8_t crc8(std::span<const std::uint8_t> bytes,std::uint8_t crc) {
    for(auto byte:bytes){crc^=byte;for(int bit=0;bit<8;++bit)crc=std::uint8_t((crc>>1)^((crc&1)?0x8c:0));}return crc;
}
std::uint16_t mcu_crc16(std::span<const std::uint8_t> bytes,std::uint16_t crc) {
    for(auto byte:bytes){crc^=byte;for(int bit=0;bit<8;++bit)crc=std::uint16_t((crc>>1)^((crc&1)?0x8408:0));}return crc;
}
std::uint16_t dm_crc16(std::span<const std::uint8_t> bytes) {
    // The deployed source shifts by one, not the standard CCITT byte update.
    // Preserve its wire compatibility until device captures establish a new protocol.
    std::uint16_t crc=0xffff;for(auto byte:bytes)crc=std::uint16_t((crc<<1)^dm_table[((crc>>8)^byte)&0xff]);return crc;
}
std::array<std::uint8_t,15> encode(const GimbalCommand& command) {
    std::array<std::uint8_t,15> frame{};frame[0]=0x5a;put32(frame.data()+1,std::bit_cast<std::uint32_t>(command.yaw));put32(frame.data()+5,std::bit_cast<std::uint32_t>(command.pitch));
    put32(frame.data()+9,std::bit_cast<std::uint32_t>(command.can_fire));const auto crc=mcu_crc16(std::span(frame).first(13));frame[13]=std::uint8_t(crc);frame[14]=std::uint8_t(crc>>8);return frame;
}
std::vector<McuFeedback> McuParser::feed(std::span<const std::uint8_t> bytes) {
    std::vector<McuFeedback> frames;
    for(auto byte:bytes) {
        buffer[size++]=byte;while(size&&buffer[0]!=0xa5)discard(buffer,size);
        if(size<buffer.size())continue;
        if(mcu_crc16(std::span(buffer).first(18))==little16(buffer.data()+18)) {
            frames.push_back({buffer[1],little_float(buffer.data()+2),little_float(buffer.data()+6),little_float(buffer.data()+10),little_float(buffer.data()+14)});++accepted;size=0;
        }else{++crc_failures;discard(buffer,size);while(size&&buffer[0]!=0xa5)discard(buffer,size);}
    }return frames;
}
std::vector<ImuSample> ImuParser::feed(std::span<const std::uint8_t> bytes) {
    std::vector<ImuSample> frames;
    for(auto byte:bytes) {
        buffer[size++]=byte;
        while(size&&(buffer[0]!=0x55||(size>=2&&buffer[1]!=0xaa))){discard(buffer,size);++discarded_bytes;}
        if(size<buffer.size())continue;
        const bool shape=buffer[18]==0x0a&&buffer[3]>=1&&buffer[3]<=3;const auto crc=little16(buffer.data()+16);
        const bool valid_crc=dm_crc16(std::span(buffer).first(16))==crc||dm_crc16(std::span(buffer).subspan(2,14))==crc;
        if(shape&&valid_crc){frames.push_back({buffer[3],{little_float(buffer.data()+4),little_float(buffer.data()+8),little_float(buffer.data()+12)}});++accepted;size=0;}
        else {if(!shape)++invalid_frames;else ++crc_failures;discard(buffer,size);}
    }return frames;
}
std::array<double,4> quaternion(const std::array<float,3>& degrees) {
    const double r=degrees[0]*std::numbers::pi/360,p=degrees[1]*std::numbers::pi/360,y=degrees[2]*std::numbers::pi/360;
    if(!std::isfinite(r+p+y))return {0,0,0,1};
    const double cr=std::cos(r),sr=std::sin(r),cp=std::cos(p),sp=std::sin(p),cy=std::cos(y),sy=std::sin(y);
    std::array<double,4> q{sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy};
    double squared=0;for(auto v:q)squared+=v*v;const double length=std::sqrt(squared);for(auto& v:q)v/=length;return q;
}
}
