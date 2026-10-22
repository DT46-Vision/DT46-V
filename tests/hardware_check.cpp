#include "modules/hardware/protocol.hpp"
#include "modules/hardware/serial_port.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>
#include <limits>

using namespace dt46::hardware;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
}
int main(int argc,char** argv) {
    try {
        if(argc!=2)throw std::invalid_argument("Expected protocol reference fixture");std::ifstream file(argv[1]);const auto fixture=nlohmann::json::parse(file);
        for(const auto& item:fixture.at("crc")) {
            const auto bytes=item.at("bytes").get<std::vector<std::uint8_t>>();require(crc8(bytes)==item.at("crc8"),"CRC8 differs from baseline");
            require(mcu_crc16(bytes)==item.at("mcu_crc16"),"MCU CRC differs from baseline");
            if(dm_crc16(bytes)!=item.at("dm_crc16"))throw std::runtime_error("DM CRC differs: length="+std::to_string(bytes.size())+" actual="+std::to_string(dm_crc16(bytes))+" expected="+item.at("dm_crc16").dump());
        }
        const auto expected=fixture.at("command").get<std::vector<std::uint8_t>>(),mcu=fixture.at("mcu").get<std::vector<std::uint8_t>>();const auto command=encode({-45.5f,12.25f,1});
        require(std::equal(expected.begin(),expected.end(),command.begin(),command.end()),"MCU command bytes differ");
        for(std::size_t split=0;split<=mcu.size();++split) {
            McuParser parser;auto a=parser.feed(std::span(mcu).first(split));const auto b=parser.feed(std::span(mcu).subspan(split));a.insert(a.end(),b.begin(),b.end());
            require(a.size()==1&&a[0].color==1&&a[0].roll==12.5f&&a[0].pitch==-23.75f&&a[0].yaw==179.f&&a[0].speed==25.25f,"Split MCU feedback changed");
        }
        std::vector<std::uint8_t> noise(20000,0x12);McuParser mcu_parser;require(mcu_parser.feed(noise).empty(),"Noise accepted as MCU feedback");
        auto corrupt=mcu;corrupt[7]^=0x80;corrupt.insert(corrupt.end(),mcu.begin(),mcu.end());require(mcu_parser.feed(corrupt).size()==1&&mcu_parser.crc_failures>0,"MCU did not recover after bad CRC");
        for(const auto& item:fixture.at("imu")) {
            const auto imu=item.get<std::vector<std::uint8_t>>();
            for(std::size_t split=0;split<=imu.size();++split) {
                ImuParser parser;parser.feed(noise);auto a=parser.feed(std::span(imu).first(split));auto b=parser.feed(std::span(imu).subspan(split));a.insert(a.end(),b.begin(),b.end());
                require(a.size()==1&&a[0].rid==3&&a[0].rpy==std::array<float,3>{12.5f,-23.75f,179.f},"Split DM feedback changed");
            }
            ImuParser parser;auto broken=imu;broken[9]^=0x80;broken.insert(broken.end(),imu.begin(),imu.end());
            require(parser.feed(broken).size()==1&&parser.crc_failures>0,"DM did not recover after bad CRC");
            auto bad_tail=imu;bad_tail.back()=0;require(parser.feed(bad_tail).empty(),"DM accepted invalid tail");
            parser.reset();auto bad_rid=imu;bad_rid[3]=0x77;require(parser.feed(bad_rid).empty(),"DM accepted invalid RID");
        }
        const auto q=quaternion({0,0,90});require(std::abs(q[2]-std::sqrt(.5))<1e-14&&std::abs(q[3]-std::sqrt(.5))<1e-14,"Euler quaternion axis/order changed");
        require(quaternion({0,std::numeric_limits<float>::infinity(),0})==std::array<double,4>{0,0,0,1},"Invalid quaternion fallback changed");
        SerialPort port;bool rejected=false;try{port.open("DT46_NONEXISTENT_TEST_PORT",115200,std::chrono::milliseconds(100));}catch(const std::exception&){rejected=true;}
        require(rejected&&!port.is_open(),"Failed port open retained a handle");
        std::cout<<"CRC fixtures, byte layouts, all frame splits, corruption recovery and quaternion checks passed\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
