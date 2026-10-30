#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "modules/media/recorder.hpp"
#include "modules/vision/debug_images.hpp"
#include "modules/vision/virtual_camera.hpp"

namespace {
dt46::Json read_json(const std::filesystem::path& path){std::ifstream input(path);if(!input)throw std::runtime_error("Cannot open "+path.string());return dt46::Json::parse(input);}
void image(const std::filesystem::path& path,const cv::Mat& rgb) {
    if(rgb.empty())return;cv::Mat bgr;if(rgb.channels()==3)cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);else bgr=rgb;
    if(!cv::imwrite(path.string(),bgr))throw std::runtime_error("Cannot write "+path.string());
}
}
int main(int argc,char** argv) {
    try {
        std::filesystem::path root=DT46_SOURCE_ROOT,sequence,output,record,parameters;std::string profile="infantry",model;double fps=20;bool dump=false;
        for(int i=1;i<argc;++i){const std::string arg=argv[i];const auto value=[&]{if(++i>=argc)throw std::invalid_argument("Missing value: "+arg);return std::string(argv[i]);};
            if(arg=="--root")root=value();else if(arg=="--sequence")sequence=value();else if(arg=="--output")output=value();else if(arg=="--record")record=value();
            else if(arg=="--parameters")parameters=value();else if(arg=="--profile")profile=value();else if(arg=="--model")model=value();else if(arg=="--video-fps")fps=std::stod(value());
            else if(arg=="--dump-debug")dump=true;else if(arg=="--help"){std::cout<<"dt46_replay --sequence frames.jsonl --output DIR [--record FILE.mp4 --video-fps 20 --profile infantry --parameters FILE.json --model FILE.onnx --dump-debug]\n";return 0;}
            else throw std::invalid_argument("Unknown argument: "+arg);}
        if(sequence.empty()||output.empty())throw std::invalid_argument("--sequence and --output are required");
        std::ifstream frames(sequence);if(!frames)throw std::runtime_error("Cannot open sequence");std::filesystem::create_directories(output);
        dt46::AimPipeline pipeline(root,profile,dump);dt46::Json changes=parameters.empty()?dt46::Json::object():read_json(parameters);
        changes.erase("camera");if(!model.empty())changes["model"]=model;if(!changes.empty())pipeline.configure(changes);
        std::unique_ptr<dt46::Recorder> recorder;if(!record.empty())recorder=std::make_unique<dt46::Recorder>(record,fps);
        std::ofstream log(output/"frames.jsonl");if(!log)throw std::runtime_error("Cannot open result log");
        std::string line;std::size_t count=0,detected=0,tracking=0,fire=0;std::uint64_t generation=UINT64_MAX;bool saved=false;cv::Mat final;dt46::Json last;
        while(std::getline(frames,line)) {
            if(line.empty())continue;auto metadata=dt46::Json::parse(line);auto path=std::filesystem::path(metadata.at("image").get<std::string>());if(path.is_relative())path=sequence.parent_path()/path;
            const cv::Mat bgr=cv::imread(path.string());if(bgr.empty())throw std::runtime_error("Cannot read "+path.string());cv::Mat rgb;cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);
            if(rgb.cols!=metadata.at("width")||rgb.rows!=metadata.at("height"))throw std::runtime_error("Frame dimensions differ from metadata");
            const auto current=metadata.at("generation").get<std::uint64_t>();if(current!=generation){if(generation!=UINT64_MAX)pipeline.reset();generation=current;}
            if(count==0&&metadata.contains("physics"))pipeline.configure({{"tracker",metadata.at("physics")}});
            dt46::VirtualFrame frame{rgb,std::move(metadata),{}};auto result=dt46::process_frame(pipeline,frame);++count;
            const bool detection=!result.at("observations").empty();detected+=detection;tracking+=result.at("state")=="TRACKING";fire+=result.at("can_fire").get<bool>();
            final=dt46::ballistic_image(dt46::tracking_image(rgb,result),result);
            if(count==1||(detection&&!saved)){image(output/"input.png",rgb);image(output/"detection.png",final);saved=detection;
                if(dump&&detection){for(const auto* view:{"result","crop_img","bin_img","model_input","img_debug","gray"})image(output/(std::string("debug_")+view+".png"),pipeline.debug_image(view));}}
            if(recorder&&recorder->due(frame.metadata.at("time"),generation)){recorder->write(final);result["recorded"]=true;}
            log<<result.dump()<<'\n';if(!log)throw std::runtime_error("Cannot write result log");last=std::move(result);
        }
        if(!frames.eof())throw std::runtime_error("Cannot read sequence");if(!count)throw std::runtime_error("Sequence contains no frames");image(output/"final.png",final);if(recorder)recorder->close();
        dt46::Json report={{"frames",count},{"frames_with_detections",detected},{"tracking_frames",tracking},{"can_fire_frames",fire},{"video_frames",recorder?recorder->frames():0},{"vision_only",true},{"last_frame",last}};
        std::ofstream summary(output/"report.json");if(!summary||!(summary<<report.dump(2)<<'\n'))throw std::runtime_error("Cannot write report");std::cout<<report.dump(2)<<'\n';return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
