#pragma once

#include <chrono>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

// Version 1: 8-byte DT46V001 magic, little-endian uint32 JSON and binary lengths,
// then UTF-8 JSON and optional contiguous RGB8 bytes. Loopback TCP only.
namespace dt46::wire {
using Json=nlohmann::json;
struct Packet {Json metadata;std::vector<std::uint8_t> bytes;};
class Disconnected : public std::runtime_error {public:using std::runtime_error::runtime_error;};
class Socket {
public:
    Socket()=default;
    explicit Socket(std::uintptr_t handle):handle_(handle){}
    ~Socket();
    Socket(Socket&& other)noexcept;Socket& operator=(Socket&& other)noexcept;
    Socket(const Socket&)=delete;Socket& operator=(const Socket&)=delete;
    static Socket connect(std::uint16_t port);
    static Socket listen(std::uint16_t& port);
    Socket accept()const;
    bool readable(std::chrono::milliseconds timeout)const;
    void send(const Json& metadata,std::span<const std::uint8_t> bytes={})const;
    Packet receive()const;
    void close();
    explicit operator bool()const{return handle_!=invalid;}
private:
    static constexpr std::uintptr_t invalid=~std::uintptr_t(0);
    std::uintptr_t handle_=invalid;
    void write_all(std::span<const std::uint8_t> bytes)const;
    void read_all(std::span<std::uint8_t> bytes)const;
};
std::string random_key();
bool key_matches(const std::string& actual,const std::string& expected);
}
