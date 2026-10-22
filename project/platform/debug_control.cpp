#include "platform/debug_control.hpp"
#include <fstream>

namespace dt46 {
DebugControl::DebugControl(const std::filesystem::path& session_file):key(wire::random_key()),listener(wire::Socket::listen(port)),session(session_file) {
    std::filesystem::create_directories(session.parent_path());std::ofstream file(session);file<<wire::Json{{"port",port},{"key",key},{"protocol",1}}.dump(2);
    if(!file)throw std::runtime_error("Cannot write debug session");
    worker=std::jthread([this](std::stop_token stop) {
        while(!stop.stop_requested())try {
            if(!listener.readable(std::chrono::milliseconds(50)))continue;auto socket=listener.accept();const auto request=socket.receive();
            if(!wire::key_matches(request.metadata.value("key",""),key)){socket.send({{"error","Invalid session key"}});continue;}
            const std::string operation=request.metadata.value("op","");
            if(operation=="show"){requested=true;socket.send({{"requested",true}});}
            else if(operation=="close"){close_requested=true;socket.send({{"requested",true}});}
            else if(operation=="save"){save_requested=true;socket.send({{"requested",true}});}
            else socket.send({{"error","Unknown debug operation"}});
        }catch(const std::exception&){/* A disconnected command client does not stop the vision task. */}
    });
}
DebugControl::~DebugControl() {
    worker.request_stop();worker.join();
    try {std::ifstream file(session);if(file&&wire::Json::parse(file).value("key","")==key){file.close();std::filesystem::remove(session);}}catch(const std::exception&){}
}
}
