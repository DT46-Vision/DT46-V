#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
#include "interfaces/transport.hpp"
#include <algorithm>
#include <array>
#include <climits>
#include <cerrno>
#include <cstring>
#include <random>
#include <stdexcept>
#include <utility>

namespace dt46::wire {
namespace {
constexpr std::size_t maximum_json=1024*1024,maximum_binary=64*1024*1024;
#ifdef _WIN32
using Handle=SOCKET;
void startup(){static const bool started=[] {WSADATA data;if(WSAStartup(MAKEWORD(2,2),&data))throw std::runtime_error("Winsock initialization failed");return true;}();(void)started;}
void close_native(Handle h){closesocket(h);}
int socket_error(){return WSAGetLastError();}
bool disconnected(int error){return error==WSAECONNRESET||error==WSAECONNABORTED||error==WSAESHUTDOWN;}
#else
using Handle=int;
void startup(){}
void close_native(Handle h){::close(h);}
int socket_error(){return errno;}
bool disconnected(int error){return error==ECONNRESET||error==EPIPE;}
#endif
void configure(Handle h) {
#ifdef _WIN32
    const DWORD timeout=2000;
    setsockopt(h,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<const char*>(&timeout),sizeof(timeout));
    setsockopt(h,SOL_SOCKET,SO_SNDTIMEO,reinterpret_cast<const char*>(&timeout),sizeof(timeout));
#else
    const timeval timeout{2,0};setsockopt(h,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(h,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
#endif
}
void check_transfer(int count,const char* operation) {
    if(count>0)return;
    if(count==0)throw Disconnected("Local endpoint closed");
    const int error=socket_error();const auto message=std::string("Local endpoint ")+operation+" failed ("+std::to_string(error)+")";
    if(disconnected(error))throw Disconnected(message);
    throw std::runtime_error(message);
}
void put32(std::uint8_t* bytes,std::uint32_t value){for(int i=0;i<4;++i)bytes[i]=std::uint8_t(value>>(8*i));}
std::uint32_t get32(const std::uint8_t* bytes){std::uint32_t result=0;for(int i=0;i<4;++i)result|=std::uint32_t(bytes[i])<<(8*i);return result;}
}
Socket::~Socket(){close();}
Socket::Socket(Socket&& other)noexcept:handle_(std::exchange(other.handle_,invalid)){}
Socket& Socket::operator=(Socket&& other)noexcept{if(this!=&other){close();handle_=std::exchange(other.handle_,invalid);}return *this;}
void Socket::close(){if(*this){close_native(static_cast<Handle>(handle_));handle_=invalid;}}
Socket Socket::connect(std::uint16_t port) {
    startup();Socket result(static_cast<std::uintptr_t>(::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP)));
    if(!result)throw std::runtime_error("Cannot create loopback socket");configure(static_cast<Handle>(result.handle_));
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(::connect(static_cast<Handle>(result.handle_),reinterpret_cast<sockaddr*>(&address),sizeof(address)))throw std::runtime_error("Cannot connect to local endpoint");return result;
}
Socket Socket::listen(std::uint16_t& port) {
    startup();Socket result(static_cast<std::uintptr_t>(::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP)));
    if(!result)throw std::runtime_error("Cannot create loopback listener");
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(bind(static_cast<Handle>(result.handle_),reinterpret_cast<sockaddr*>(&address),sizeof(address))||::listen(static_cast<Handle>(result.handle_),4))throw std::runtime_error("Cannot bind loopback listener");
#ifdef _WIN32
    int size=sizeof(address);
#else
    socklen_t size=sizeof(address);
#endif
    if(getsockname(static_cast<Handle>(result.handle_),reinterpret_cast<sockaddr*>(&address),&size))throw std::runtime_error("Cannot inspect listener port");port=ntohs(address.sin_port);return result;
}
Socket Socket::accept()const {
    Socket result(static_cast<std::uintptr_t>(::accept(static_cast<Handle>(handle_),nullptr,nullptr)));
    if(!result)throw std::runtime_error("Local endpoint accept failed");configure(static_cast<Handle>(result.handle_));return result;
}
bool Socket::readable(std::chrono::milliseconds timeout)const {
    if(!*this)throw std::runtime_error("Socket closed");fd_set reads;FD_ZERO(&reads);FD_SET(static_cast<Handle>(handle_),&reads);
    timeval time{long(timeout.count()/1000),long(timeout.count()%1000*1000)};
    const int status=select(int(handle_)+1,&reads,nullptr,nullptr,&time);if(status<0)throw std::runtime_error("Local endpoint select failed");return status>0;
}
void Socket::write_all(std::span<const std::uint8_t> bytes)const {
    while(!bytes.empty()) {
        const int amount=int(std::min(bytes.size(),std::size_t(INT_MAX)));
#ifdef _WIN32
        const auto count=::send(static_cast<Handle>(handle_),reinterpret_cast<const char*>(bytes.data()),amount,0);
#else
        const auto count=::send(static_cast<Handle>(handle_),bytes.data(),amount,MSG_NOSIGNAL);
#endif
        check_transfer(int(count),"send");bytes=bytes.subspan(std::size_t(count));
    }
}
void Socket::read_all(std::span<std::uint8_t> bytes)const {
    while(!bytes.empty()) {
        const auto count=::recv(static_cast<Handle>(handle_),reinterpret_cast<char*>(bytes.data()),int(std::min(bytes.size(),std::size_t(INT_MAX))),0);
        check_transfer(int(count),"receive");bytes=bytes.subspan(std::size_t(count));
    }
}
void Socket::send(const Json& metadata,std::span<const std::uint8_t> bytes)const {
    const std::string json=metadata.dump();
    if(json.size()>maximum_json||bytes.size()>maximum_binary)throw std::invalid_argument("Transport packet exceeds size limit");
    std::array<std::uint8_t,16> header{};std::memcpy(header.data(),"DT46V001",8);put32(header.data()+8,std::uint32_t(json.size()));put32(header.data()+12,std::uint32_t(bytes.size()));
    write_all(header);write_all({reinterpret_cast<const std::uint8_t*>(json.data()),json.size()});write_all(bytes);
}
Packet Socket::receive()const {
    std::array<std::uint8_t,16> header{};read_all(header);
    if(std::memcmp(header.data(),"DT46V001",8))throw std::invalid_argument("Unsupported transport protocol");
    const auto json_size=get32(header.data()+8),binary_size=get32(header.data()+12);
    if(!json_size||json_size>maximum_json||binary_size>maximum_binary)throw std::invalid_argument("Invalid transport packet lengths");
    std::string json(json_size,'\0');read_all({reinterpret_cast<std::uint8_t*>(json.data()),json.size()});
    Packet result{Json::parse(json),std::vector<std::uint8_t>(binary_size)};read_all(result.bytes);return result;
}
std::string random_key(){std::random_device random;constexpr char hex[]="0123456789abcdef";std::string key;for(int i=0;i<32;++i){const auto b=random()&255;key+=hex[b>>4];key+=hex[b&15];}return key;}
bool key_matches(const std::string& actual,const std::string& expected){if(actual.size()!=expected.size())return false;unsigned difference=0;for(std::size_t i=0;i<actual.size();++i)difference|=static_cast<unsigned char>(actual[i])^static_cast<unsigned char>(expected[i]);return difference==0;}
}
