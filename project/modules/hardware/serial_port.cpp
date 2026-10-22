#include "modules/hardware/serial_port.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <system_error>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace dt46::hardware {
namespace {
[[noreturn]] void fail(const char* operation) {
#ifdef _WIN32
    throw std::system_error(int(GetLastError()),std::system_category(),operation);
#else
    throw std::system_error(errno,std::generic_category(),operation);
#endif
}
#ifndef _WIN32
speed_t speed(int baud) {
    switch(baud){case 9600:return B9600;case 19200:return B19200;case 38400:return B38400;case 57600:return B57600;case 115200:return B115200;
#ifdef B230400
    case 230400:return B230400;
#endif
#ifdef B460800
    case 460800:return B460800;
#endif
#ifdef B921600
    case 921600:return B921600;
#endif
    default:throw std::invalid_argument("Unsupported serial baud rate");}
}
#endif
}
SerialPort::~SerialPort(){close();}
void SerialPort::close()noexcept {
    if(!is_open())return;
#ifdef _WIN32
    CloseHandle(reinterpret_cast<HANDLE>(handle));
#else
    ::close(int(handle));
#endif
    handle=-1;
}
void SerialPort::open(const std::string& port,int baud,std::chrono::milliseconds timeout) {
    if(baud<=0||timeout.count()<0||timeout.count()>60000)throw std::invalid_argument("Invalid serial baud/timeout");close();write_timeout=timeout;
    try {
#ifdef _WIN32
        const std::string path=port.starts_with("\\\\.\\")?port:"\\\\.\\"+port;
        const auto h=CreateFileA(path.c_str(),GENERIC_READ|GENERIC_WRITE,0,nullptr,OPEN_EXISTING,0,nullptr);if(h==INVALID_HANDLE_VALUE)fail("Open serial port");handle=reinterpret_cast<std::intptr_t>(h);
        DCB settings{};settings.DCBlength=sizeof(settings);if(!GetCommState(h,&settings))fail("Read serial settings");
        settings.BaudRate=DWORD(baud);settings.ByteSize=8;settings.Parity=NOPARITY;settings.StopBits=ONESTOPBIT;settings.fBinary=TRUE;settings.fParity=FALSE;
        settings.fOutxCtsFlow=settings.fOutxDsrFlow=FALSE;settings.fDtrControl=DTR_CONTROL_ENABLE;settings.fRtsControl=RTS_CONTROL_ENABLE;settings.fOutX=settings.fInX=FALSE;settings.fDsrSensitivity=FALSE;settings.fAbortOnError=FALSE;
        if(!SetCommState(h,&settings))fail("Set serial settings");COMMTIMEOUTS times{};times.ReadIntervalTimeout=MAXDWORD;times.WriteTotalTimeoutConstant=DWORD(timeout.count());
        if(!SetCommTimeouts(h,&times)||!PurgeComm(h,PURGE_RXCLEAR))fail("Set serial timeout");
#else
        const auto rate=speed(baud);handle=::open(port.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);if(handle<0)fail("Open serial port");
        termios settings{};if(tcgetattr(int(handle),&settings))fail("Read serial settings");cfmakeraw(&settings);settings.c_cflag|=CLOCAL|CREAD;settings.c_cflag&=~(PARENB|CSTOPB|CSIZE);
        settings.c_cflag|=CS8;
#ifdef CRTSCTS
        settings.c_cflag&=~CRTSCTS;
#endif
        settings.c_cc[VMIN]=0;settings.c_cc[VTIME]=0;
        if(cfsetispeed(&settings,rate)||cfsetospeed(&settings,rate)||tcsetattr(int(handle),TCSANOW,&settings)||tcflush(int(handle),TCIFLUSH))fail("Set serial settings");
#endif
    }catch(...){close();throw;}
}
std::size_t SerialPort::read(std::span<std::uint8_t> buffer) {
    if(!is_open())throw std::runtime_error("Serial port is closed");
#ifdef _WIN32
    DWORD errors=0;COMSTAT status{};if(!ClearCommError(reinterpret_cast<HANDLE>(handle),&errors,&status))fail("Inspect serial port");
    DWORD count=0;const auto available=DWORD(std::min<std::size_t>(buffer.size(),status.cbInQue));
    if(available&&!ReadFile(reinterpret_cast<HANDLE>(handle),buffer.data(),available,&count,nullptr))fail("Read serial port");return count;
#else
    pollfd descriptor{int(handle),POLLIN,0};const int available=poll(&descriptor,1,0);
    if(available<0){if(errno==EINTR)return 0;fail("Poll serial port");}
    if(descriptor.revents&(POLLHUP|POLLERR|POLLNVAL))throw std::runtime_error("Serial port disconnected");if(!available)return 0;
    const auto count=::read(int(handle),buffer.data(),buffer.size());if(count<0){if(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)return 0;fail("Read serial port");}return std::size_t(count);
#endif
}
void SerialPort::write(std::span<const std::uint8_t> bytes) {
    if(!is_open())throw std::runtime_error("Serial port is closed");const auto deadline=std::chrono::steady_clock::now()+write_timeout;
    while(!bytes.empty()) {
#ifdef _WIN32
        DWORD count=0;if(!WriteFile(reinterpret_cast<HANDLE>(handle),bytes.data(),DWORD(std::min<std::size_t>(bytes.size(),MAXDWORD)),&count,nullptr))fail("Write serial port");
#else
        const auto remaining=std::max(std::chrono::milliseconds(0),std::chrono::duration_cast<std::chrono::milliseconds>(deadline-std::chrono::steady_clock::now()));
        pollfd descriptor{int(handle),POLLOUT,0};const int ready=poll(&descriptor,1,int(remaining.count()));if(ready<0){if(errno==EINTR)continue;fail("Poll serial output");}
        if(!ready)throw std::runtime_error("Serial write timed out");
        const auto count=::write(int(handle),bytes.data(),bytes.size());if(count<0){if(errno==EAGAIN||errno==EINTR)continue;fail("Write serial port");}
#endif
        if(!count)throw std::runtime_error("Serial write made no progress");bytes=bytes.subspan(std::size_t(count));
        if(!bytes.empty()&&std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("Serial write timed out");
    }
}
}
