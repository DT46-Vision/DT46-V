#pragma once
#include <chrono>
#include <cstdint>
#include <span>
#include <string>

namespace dt46::hardware {
// One I/O thread owns each port; reconnect closes it on that same thread.
class SerialPort {
public:
    SerialPort()=default;~SerialPort();
    SerialPort(const SerialPort&)=delete;SerialPort& operator=(const SerialPort&)=delete;
    void open(const std::string& port,int baud,std::chrono::milliseconds write_timeout);
    void close()noexcept;
    bool is_open()const{return handle!=-1;}
    std::size_t read(std::span<std::uint8_t> buffer);
    void write(std::span<const std::uint8_t> bytes);
private:
    std::intptr_t handle=-1;std::chrono::milliseconds write_timeout{1000};
};
}
