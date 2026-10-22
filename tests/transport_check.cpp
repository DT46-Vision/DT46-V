#include "interfaces/transport.hpp"
#include <future>
#include <iostream>

int main() {
    try {
        using namespace dt46::wire;
        std::uint16_t port=0;auto listener=Socket::listen(port);const auto key=random_key();
        auto server=std::async(std::launch::async,[&] {
            auto socket=listener.accept();const auto hello=socket.receive();
            if(!key_matches(hello.metadata.at("key"),key))throw std::runtime_error("Authentication failed");
            auto packet=socket.receive();socket.send(packet.metadata,packet.bytes);
        });
        auto client=Socket::connect(port);client.send({{"key",key}});
        std::vector<std::uint8_t> rgb(1280*1024*3);for(std::size_t i=0;i<rgb.size();++i)rgb[i]=std::uint8_t(i%251);
        const Json metadata={{"time",.123},{"width",1280},{"height",1024},{"color_order","RGB"},{"generation",3}};
        client.send(metadata,rgb);auto echo=client.receive();server.get();
        if(echo.metadata!=metadata||echo.bytes!=rgb)throw std::runtime_error("RGB packet changed during transport");
        if(key_matches("bad",key)||key_matches(key.substr(1)+"0",key))throw std::runtime_error("Incorrect key accepted");
        bool oversized=false;try{client.send({{"padding",std::string(1024*1024,'x')}});}catch(const std::invalid_argument&){oversized=true;}
        if(!oversized)throw std::runtime_error("Oversized metadata accepted");
        std::cout<<"Full-resolution RGB, metadata, authentication and size limits passed\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
