#include "modules/vision/debug_window.hpp"
#include "platform/debug_control.hpp"
#include "modules/media/recorder.hpp"
#include "modules/vision/debug_images.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

namespace {
using Clock=std::chrono::steady_clock;
using dt46::Json;
Json read_json(const std::filesystem::path& path){std::ifstream input(path);if(!input)throw std::runtime_error("Cannot open "+path.string());return Json::parse(input);}
void write_json(const std::filesystem::path& path,const Json& value){std::ofstream output(path);if(!output||!(output<<value.dump(2)<<'\n'))throw std::runtime_error("Cannot write "+path.string());}
struct Options {
    std::filesystem::path root=DT46_SOURCE_ROOT,session,debug_session,output,record,parameters;
    std::string profile="infantry",command;
    double duration=0,video_fps=20;std::uint64_t maximum_frames=0;bool hidden=false,debug=false,help=false;
};
Options options(int argc,char** argv) {
    Options o;
    for(int i=1;i<argc;++i) {
        const std::string arg=argv[i];const auto value=[&]{if(i+1>=argc)throw std::invalid_argument("Missing value for "+arg);return std::string(argv[++i]);};
        if(arg=="--root")o.root=value();else if(arg=="--camera-session")o.session=value();else if(arg=="--output")o.output=value();
        else if(arg=="--debug-session")o.debug_session=value();else if(arg=="--profile")o.profile=value();
        else if(arg=="--duration")o.duration=std::stod(value());else if(arg=="--frames")o.maximum_frames=std::stoull(value());
        else if(arg=="--record")o.record=value();else if(arg=="--video-fps")o.video_fps=std::stod(value());else if(arg=="--parameters")o.parameters=value();
        else if(arg=="--show-debug")o.command="show";else if(arg=="--close-debug")o.command="close";
        else if(arg=="--save-debug")o.command="save";else if(arg=="--debug")o.debug=true;
        else if(arg=="--debug-hidden")o.hidden=true;else if(arg=="--help")o.help=true;
        else throw std::invalid_argument("Unknown argument: "+arg);
    }
    if(!std::isfinite(o.duration)||o.duration<0)throw std::invalid_argument("Duration must be finite and nonnegative");
    if(!std::isfinite(o.video_fps)||o.video_fps<=0)throw std::invalid_argument("Video FPS must be positive and finite");
    if(o.output.empty())o.output=o.root/".cache/virtual_camera";
    if(o.debug_session.empty())o.debug_session=o.root/".cache/virtual_camera/session.json";
    return o;
}
int send_command(const Options& o) {
    const auto session=read_json(o.debug_session);auto socket=dt46::wire::Socket::connect(session.at("port"));
    socket.send({{"key",session.at("key")},{"op",o.command}});const auto reply=socket.receive();
    if(reply.metadata.contains("error"))throw std::runtime_error(reply.metadata.at("error"));
    std::cout<<reply.metadata.dump()<<'\n';return 0;
}
class Session {
public:
    explicit Session(const Options& options):o(options),camera(read_json(o.session)),pipeline(o.root,o.profile,true),control(o.debug_session) {
        auto settings=o.parameters.empty()?Json::object():read_json(o.parameters);
        if(settings.is_object()&&settings.empty())settings=read_json(o.root/"standalone/virtual_camera.json");
        if(settings.contains("camera")){camera.validate(settings.at("camera"));camera.configure(settings.at("camera"));settings.erase("camera");}
        for(const auto& [key,value]:camera.description.at("physics").items())settings["tracker"][key]=value;
        pipeline.configure(settings);
        if(!o.record.empty())recorder=std::make_unique<dt46::Recorder>(o.record,o.video_fps);
        std::filesystem::create_directories(o.output);log.open(o.output/"frames.jsonl");events.open(o.output/"debug_events.jsonl");
        if(!log||!events)throw std::runtime_error("Cannot open vision logs");control.requested=o.debug;
    }
    int run() {
        std::string stopped="limit",error;
        try {
            while((o.duration<=0||elapsed()<o.duration)&&(!o.maximum_frames||frames<o.maximum_frames)) {
                debug_commands();
                auto frame=camera.next({{"frames",frames},{"debug_open",bool(debug)},{"context",context()},
                    {"state",result.is_null()?"STARTING":result.at("state").get<std::string>()}},
                    result_revision==pipeline.revision()?dt46::aim_feedback(result):Json(nullptr));
                if(frame)process(*frame);else std::this_thread::sleep_for(std::chrono::milliseconds(1));
                const auto now=Clock::now();
                if(debug&&now-last_draw>=std::chrono::milliseconds(33)) {
                    try{if(!debug->pump())close_debug();}catch(const std::exception& e){event("window_error",e.what());close_debug();}last_draw=now;
                }
            }
        }catch(const dt46::wire::Disconnected& e){stopped=e.what();}
        catch(const std::exception& e){stopped="error";error=e.what();}
        close_debug();if(recorder)recorder->close();log.flush();events.flush();
        write_json(o.output/"report.json",{{"frames",frames},{"wall_seconds",elapsed()},{"last_frame",result},
            {"stopped",stopped},{"error",error},{"profile",o.profile},{"context",context()},{"video_frames",recorder?recorder->frames():0}});
        if(!error.empty())throw std::runtime_error(error);
        std::cout<<"Processed "<<frames<<" virtual-camera frames\n";return 0;
    }
private:
    const Options& o;dt46::VirtualCamera camera;dt46::AimPipeline pipeline;dt46::DebugControl control;
    std::unique_ptr<dt46::VisionDebug> debug;std::ofstream log,events;
    std::unique_ptr<dt46::Recorder> recorder;
    std::uint64_t frames=0,generation=UINT64_MAX;double last_time=-1;
    std::uint64_t result_revision=0;
    Json result=nullptr;Clock::time_point began=Clock::now(),last_draw{},last_images{},last_fps=began;std::uint64_t fps_frames=0;
    std::uintptr_t context()const{return reinterpret_cast<std::uintptr_t>(pipeline.context());}
    double elapsed()const{return std::chrono::duration<double>(Clock::now()-began).count();}
    void event(const std::string& operation,const std::string& error="") {
        events<<Json{{"event",operation},{"frames",frames},{"context",context()},{"wall_seconds",elapsed()},{"error",error}}.dump()<<'\n';events.flush();
    }
    void close_debug(){if(debug){debug.reset();event("closed");}}
    void debug_commands() {
        if(control.close_requested.exchange(false))close_debug();
        try {
            if(control.requested.exchange(false)) {
                if(!debug){debug=std::make_unique<dt46::VisionDebug>(pipeline,camera,o.output,!o.hidden);event("opened");}
                else if(!o.hidden)debug->show();
            }
            if(control.save_requested.exchange(false)&&debug){debug->save_frame();event("saved");}
        }catch(const std::exception& e){event("window_error",e.what());std::cerr<<"Debug window: "<<e.what()<<'\n';}
    }
    void process(const dt46::VirtualFrame& frame) {
        const auto next_generation=frame.metadata.at("generation").get<std::uint64_t>();
        if(next_generation!=generation){if(generation!=UINT64_MAX)pipeline.reset();generation=next_generation;last_time=-1;}
        const double timestamp=frame.metadata.at("time");if(timestamp<=last_time)return;
        result=dt46::process_frame(pipeline,frame);result_revision=pipeline.revision();last_time=timestamp;++frames;
        const auto& display=pipeline.settings().at("tracker");
        const auto enabled=[](const Json& value){return value.is_boolean()?value.get<bool>():value.get<double>()!=0;};
        if(recorder&&enabled(display.at("display"))&&recorder->due(timestamp,generation)){
            const double size=display.at("text_size");recorder->write(dt46::ballistic_image(dt46::tracking_image(frame.rgb,result,size),result,size));result["recorded"]=true;
        }
        log<<result.dump()<<'\n';
        if(!log)throw std::runtime_error("Cannot write vision frame log");
        const auto now=Clock::now();const double seconds=std::chrono::duration<double>(now-last_fps).count();
        if(seconds*1000>=pipeline.settings().at("detector").at("print_period_ms").get<double>()){
            std::cout<<"[vision] processed FPS="<<(frames-fps_frames)/seconds<<'\n';last_fps=now;fps_frames=frames;
        }
        if(debug&&(!enabled(display.at("display_fps_limit"))||now-last_images>=std::chrono::milliseconds(33))) {
            try{debug->update(frame,result);}catch(const std::exception& e){event("window_error",e.what());close_debug();}last_images=now;
        }
    }
};
}
int main(int argc,char** argv) {
    try {
        const auto o=options(argc,argv);
        if(o.help){std::cout<<"dt46_vision --camera-session FILE [--root DIR] [--profile infantry|sentry|hero] [--output DIR] [--duration SECONDS] [--frames COUNT] [--debug] [--debug-hidden] [--parameters FILE] [--record FILE.mp4 --video-fps 20]\n"
            <<"dt46_vision --show-debug|--close-debug|--save-debug [--root DIR] [--debug-session FILE]\n";return 0;}
        if(!o.command.empty())return send_command(o);
        if(o.session.empty())throw std::invalid_argument("--camera-session is required");
        return Session(o).run();
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
