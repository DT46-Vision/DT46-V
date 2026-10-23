#define GLAD_GL_IMPLEMENTATION
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#define NK_IMPLEMENTATION
#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_STANDARD_VARARGS
#define NK_BUTTON_TRIGGER_ON_RELEASE
#include <nuklear.h>
#define NK_GLFW_GL2_IMPLEMENTATION
#include <nuklear_glfw_gl2.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "modules/vision/debug_window.hpp"
#include "modules/vision/debug_images.hpp"
#include <opencv2/imgproc.hpp>
#include <array>
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>
#endif

namespace dt46 {
namespace {
std::array<const char*,10> views={"image_raw","ballistic_img","tracking_state_img","result","crop_img","bin_img","img_armor","img_armor_processed","img_debug","gray"};
bool physics_input(const std::string& group,const std::string& key){return group=="tracker"&&(key=="k_v2"||key=="gravity"||key=="bullet_speed");}
void png(const std::filesystem::path& path,const cv::Mat& rgb) {
    std::ofstream file(path,std::ios::binary);const auto callback=[](void* context,void* data,int size){static_cast<std::ofstream*>(context)->write(static_cast<char*>(data),size);};
    if(!file||!stbi_write_png_to_func(callback,&file,rgb.cols,rgb.rows,rgb.channels(),rgb.data,int(rgb.step))||!file)throw std::runtime_error("Cannot save "+path.string());
}
void json_file(const std::filesystem::path& path,const Json& value){std::ofstream file(path);if(!file||!(file<<value.dump(2)<<'\n'))throw std::runtime_error("Cannot save "+path.string());}
std::filesystem::path choose_file(bool save,const std::filesystem::path& fallback) {
#ifdef _WIN32
    std::array<wchar_t,32768> name{};const auto initial=fallback.wstring();std::copy(initial.begin(),initial.end(),name.begin());
    OPENFILENAMEW dialog{};dialog.lStructSize=sizeof(dialog);dialog.lpstrFile=name.data();dialog.nMaxFile=DWORD(name.size());dialog.lpstrFilter=L"Vision parameters (*.json)\0*.json\0All files\0*.*\0\0";dialog.lpstrDefExt=L"json";
    dialog.Flags=OFN_EXPLORER|OFN_PATHMUSTEXIST|OFN_NOCHANGEDIR|(save?OFN_OVERWRITEPROMPT:OFN_FILEMUSTEXIST);
    if(save?GetSaveFileNameW(&dialog):GetOpenFileNameW(&dialog))return std::filesystem::path(name.data());return {};
#else
    return fallback;
#endif
}
}
struct VisionDebug::Impl {
    AimPipeline& pipeline;VirtualCamera& camera;std::filesystem::path output;
    GLFWwindow* window=nullptr;nk_context* ui=nullptr;bool backend=false;
    VirtualFrame last_frame;Json last_result=nullptr;std::array<cv::Mat,10> images;std::array<GLuint,4> textures{};
    std::array<int,4> choices{3,1,7,5};std::array<int,4> texture_views{-1,-1,-1,-1};bool dirty=true;
    struct ViewOptions {int smooth=0,zoom=0,grid=0,rotation=0;};std::array<ViewOptions,4> view_options;
    std::map<std::string,std::map<std::string,std::array<char,256>>> fields;Json field_types;
    std::string status="Starting",message;std::deque<std::string> logs;int tab=0;std::filesystem::path pending_screenshot;
    Impl(AimPipeline& p,VirtualCamera& c,std::filesystem::path folder,bool visible):pipeline(p),camera(c),output(std::move(folder)) {
        if(!glfwInit())throw std::runtime_error("Cannot initialize vision debug window");
        glfwDefaultWindowHints();glfwWindowHint(GLFW_VISIBLE,visible?GLFW_TRUE:GLFW_FALSE);glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_COMPAT_PROFILE);
        window=glfwCreateWindow(1440,900,"DT46 - Native vision debug",nullptr,nullptr);
        if(!window){glfwTerminate();throw std::runtime_error("Cannot create vision debug window");}
        glfwMakeContextCurrent(window);if(!gladLoadGL(glfwGetProcAddress)){glfwDestroyWindow(window);glfwTerminate();throw std::runtime_error("Cannot initialize debug OpenGL");}
        glfwSwapInterval(0);ui=nk_glfw3_init(window,NK_GLFW3_INSTALL_CALLBACKS);backend=true;
        nk_font_atlas* atlas;nk_glfw3_font_stash_begin(&atlas);nk_glfw3_font_stash_end();glGenTextures(4,textures.data());
        for(GLuint texture:textures){glBindTexture(GL_TEXTURE_2D,texture);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);}
        Json groups={{"camera",camera.description.at("camera")},{"detector",pipeline.settings().at("detector")},{"tracker",pipeline.settings().at("tracker")}};
        groups["detector"]["cls_model_file"]=pipeline.settings().at("model");field_types=groups;
        for(const auto& [group,values]:groups.items())for(const auto& [key,value]:values.items()) {
            const std::string text=value.is_string()?value.get<std::string>():value.is_boolean()?(value.get<bool>()?"1":"0"):value.dump();auto& buffer=fields[group][key];std::copy_n(text.c_str(),std::min(text.size(),buffer.size()-1),buffer.data());
        }
    }
    ~Impl(){if(window){glfwMakeContextCurrent(window);glDeleteTextures(4,textures.data());if(backend)nk_glfw3_shutdown();glfwDestroyWindow(window);glfwTerminate();}}
    Json settings()const {
        Json result=Json::object();for(const auto& [group,values]:fields)for(const auto& [key,text]:values) {
            if(key=="cls_model_file"){result["model"]=text.data();continue;}
            std::size_t consumed=0;const std::string value=text.data();const double number=std::stod(value,&consumed);
            if(consumed!=value.size()||!std::isfinite(number))throw std::invalid_argument("Invalid parameter: "+key);
            if(field_types.at(group).at(key).is_boolean()){
                if(number!=0&&number!=1)throw std::invalid_argument("Expected boolean: "+key);result[group][key]=number!=0;
            }else result[group][key]=number;
        }return result;
    }
    void apply() {
        const auto values=settings();camera.validate(values.at("camera"));Json tracker=values.at("tracker");for(const char* key:{"k_v2","gravity","bullet_speed"})tracker.erase(key);
        pipeline.configure({{"model",values.at("model")},{"detector",values.at("detector")},{"tracker",tracker}});camera.configure(values.at("camera"));message="Applied; tracker reset; source configuration unchanged";
    }
    void load() {
        const auto path=choose_file(false,output/"vision_parameters.json");if(path.empty())return;
        std::ifstream input(path);if(!input)throw std::runtime_error("Cannot open parameters");const auto values=Json::parse(input);if(!values.is_object())throw std::invalid_argument("Parameters must be an object");
        if(values.contains("camera"))camera.validate(values.at("camera"));
        for(auto& [group,entries]:fields)for(auto& [key,buffer]:entries)if(!physics_input(group,key)) {
            const bool model=key=="cls_model_file"&&values.contains("model");
            if(!model&&(!values.contains(group)||!values.at(group).contains(key)))continue;
            const auto& value=model?values.at("model"):values.at(group).at(key);const std::string text=value.is_string()?value.get<std::string>():value.is_boolean()?(value.get<bool>()?"1":"0"):value.dump();if(text.size()>=buffer.size())throw std::invalid_argument("Parameter text is too long");buffer.fill(0);std::copy(text.begin(),text.end(),buffer.begin());
        }apply();
    }
    void save_settings(){const auto path=choose_file(true,output/"vision_parameters.json");if(!path.empty()){json_file(path,settings());message="Saved "+path.string();}}
    void save_frame() {
        if(last_result.is_null())return;std::ostringstream name;name<<"frame_"<<std::fixed<<std::setprecision(6)<<last_result.at("simulation_time_s").get<double>();const auto folder=output/name.str();std::filesystem::create_directories(folder);
        for(std::size_t i=0;i<views.size();++i)if(!images[i].empty())png(folder/(std::string(views[i])+".png"),images[i]);json_file(folder/"result.json",last_result);json_file(folder/"parameters.json",settings());message="Saved "+name.str();pending_screenshot=folder/"window.png";
    }
    void update(const VirtualFrame& frame,const Json& result) {
        last_frame=frame;last_result=result;images[0]=frame.rgb;const auto& tracker=pipeline.settings().at("tracker");const auto& display=tracker.at("display");
        if(display.is_boolean()?display.get<bool>():display.get<double>()!=0){const double size=tracker.at("text_size");images[1]=ballistic_image(frame.rgb,result,size);images[2]=tracking_image(frame.rgb,result,size);}
        else{images[1].release();images[2].release();}
        for(std::size_t i=3;i<views.size();++i) {
            images[i]=pipeline.debug_image(views[i]);
            if(!images[i].empty()&&images[i].channels()==1)cv::cvtColor(images[i],images[i],cv::COLOR_GRAY2RGB);
        }dirty=true;
        std::ostringstream text;text<<"t="<<std::fixed<<std::setprecision(2)<<result.at("simulation_time_s").get<double>()<<"s  "<<result.at("state").get<std::string>()<<" id="<<result.at("tracked_id").get<int>()<<" fire="<<result.at("can_fire").get<bool>()<<" vision="<<std::setprecision(1)<<result.at("processing_ms").get<double>()<<"ms";status=text.str();
        for(const auto& log:result.at("tracker_debug").at("logs")){logs.push_back(std::to_string(result.at("simulation_time_s").get<double>())+" "+log.at("name").get<std::string>()+": "+log.at("value").dump());if(logs.size()>1000)logs.pop_front();}
    }
    void pane(int index,float x,float y,float width,float height) {
        const std::string title="View "+std::to_string(index+1);
        if(nk_begin(ui,title.c_str(),nk_rect(x,y,width,height),NK_WINDOW_BORDER|NK_WINDOW_NO_SCROLLBAR)) {
            nk_layout_row_dynamic(ui,24,1);choices[index]=nk_combo(ui,views.data(),int(views.size()),choices[index],22,nk_vec2(230,250));
            auto& options=view_options[index];const auto previous=options;
            nk_layout_row_dynamic(ui,22,4);nk_checkbox_label(ui,"Smooth",&options.smooth);nk_checkbox_label(ui,"1:1",&options.zoom);nk_checkbox_label(ui,"Grid",&options.grid);
            const char* rotations[]={"0 deg","90 deg","180 deg","270 deg"};options.rotation=nk_combo(ui,rotations,4,options.rotation,22,nk_vec2(110,125));
            const auto& source=images[choices[index]];cv::Mat image=source;
            if(!source.empty()&&options.rotation)cv::rotate(source,image,options.rotation==1?cv::ROTATE_90_CLOCKWISE:options.rotation==2?cv::ROTATE_180:cv::ROTATE_90_COUNTERCLOCKWISE);
            if(image.empty()){nk_layout_row_dynamic(ui,30,1);nk_label(ui,"No current image / display disabled",NK_TEXT_CENTERED);}
            else {
                glBindTexture(GL_TEXTURE_2D,textures[index]);const int filter=options.smooth?GL_LINEAR:GL_NEAREST;
                glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,filter);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,filter);
                if(dirty||texture_views[index]!=choices[index]||previous.rotation!=options.rotation) {
                    glBindTexture(GL_TEXTURE_2D,textures[index]);glPixelStorei(GL_UNPACK_ALIGNMENT,1);glPixelStorei(GL_UNPACK_ROW_LENGTH,0);
                    glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,image.cols,image.rows,0,GL_RGB,GL_UNSIGNED_BYTE,image.data);texture_views[index]=choices[index];
                }
                const float available_w=width-28,available_h=std::max(1.f,height-87),scale=options.zoom?1.f:std::min(available_w/image.cols,available_h/image.rows);
                nk_layout_row_dynamic(ui,available_h,1);const std::string group=title+" image";
                if(nk_group_begin(ui,group.c_str(),options.zoom?0:NK_WINDOW_NO_SCROLLBAR)){
                    nk_layout_row_static(ui,image.rows*scale,int(image.cols*scale),1);const auto bounds=nk_widget_bounds(ui);nk_image(ui,nk_image_id(int(textures[index])));
                    if(options.grid){auto* canvas=nk_window_get_canvas(ui);for(int line=1;line<4;++line){const float gx=bounds.x+bounds.w*line/4,gy=bounds.y+bounds.h*line/4;
                        nk_stroke_line(canvas,gx,bounds.y,gx,bounds.y+bounds.h,1,nk_rgba(255,255,255,150));nk_stroke_line(canvas,bounds.x,gy,bounds.x+bounds.w,gy,1,nk_rgba(255,255,255,150));}}
                    nk_group_end(ui);
                }
            }
        }nk_end(ui);
    }
    void parameters(float x,float y,float width,float height) {
        if(nk_begin(ui,"Settings",nk_rect(x,y,width,height),NK_WINDOW_BORDER|NK_WINDOW_NO_SCROLLBAR)) {
            nk_layout_row_dynamic(ui,24,1);nk_label(ui,"Simulated camera / original vision",NK_TEXT_LEFT);
            constexpr const char* groups[]={"camera","detector","tracker"};nk_layout_row_dynamic(ui,24,3);
            for(int i=0;i<3;++i)if(nk_option_label(ui,groups[i],tab==i))tab=i;
            nk_layout_row_dynamic(ui,std::max(120.f,height-315),1);
            if(nk_group_begin(ui,"Parameters",NK_WINDOW_BORDER)) {
                for(auto& [key,buffer]:fields.at(groups[tab])) {
                    nk_layout_row_dynamic(ui,24,1);nk_label(ui,key.c_str(),NK_TEXT_LEFT);
                    nk_layout_row_dynamic(ui,24,1);
                    if(physics_input(groups[tab],key))nk_label(ui,buffer.data(),NK_TEXT_LEFT);
                    else if(field_types.at(groups[tab]).at(key).is_boolean()){
                        int enabled=std::string(buffer.data())=="1"||std::string(buffer.data())=="1.0";
                        if(nk_checkbox_label(ui,"Enabled",&enabled)){buffer.fill(0);buffer[0]=enabled?'1':'0';}
                    }else nk_edit_string_zero_terminated(ui,NK_EDIT_FIELD,buffer.data(),int(buffer.size()),key=="cls_model_file"?nk_filter_default:nk_filter_float);
                }
                const auto note=[&](const std::string& text){nk_layout_row_dynamic(ui,42,1);nk_label_wrap(ui,text.c_str());};
                if(tab==0){note("View rotation changes display only; vision uses the original camera image");note("camera_name / camera_info_url: frame camera and K/D; use_sensor_data_qos: ROS only");}
                if(tab==1){note("detect_color: -1 follow target, 0 red, 1 blue, 2 both");note("use_geometric_center: false (frame calibration)");note("print_period_ms: native processing FPS log interval");}
                if(tab==2){note("target_color: 0 red / 1 blue; follow_decision uses simulation feedback");
                    if(!last_result.is_null())note("cam_to_gun_pos [m], feedback: "+last_result.at("input").at("muzzle_offset_camera_m").dump());
                    note("rotation_rpy_*: ROS IMU adapter replaced by optical feedback");}
                nk_group_end(ui);
            }
            nk_layout_row_dynamic(ui,20,1);nk_label(ui,"k / gravity / speed follow shooter config",NK_TEXT_LEFT);
            const auto action=[&](const char* label,auto fn){nk_layout_row_dynamic(ui,27,1);if(nk_button_label(ui,label))try{fn();}catch(const std::exception& e){message=e.what();}};
            action("Apply (resets tracker)",[&]{apply();});action("Load parameters JSON",[&]{load();});action("Save parameters JSON",[&]{save_settings();});
            nk_layout_row_dynamic(ui,44,1);nk_label_wrap(ui,message.c_str());nk_layout_row_dynamic(ui,100,1);
            if(nk_group_begin(ui,"Logs",NK_WINDOW_BORDER)){for(const auto& line:logs){nk_layout_row_dynamic(ui,18,1);nk_label(ui,line.c_str(),NK_TEXT_LEFT);}nk_group_end(ui);}
        }nk_end(ui);
    }
    bool pump() {
        glfwMakeContextCurrent(window);glfwPollEvents();if(glfwWindowShouldClose(window))return false;nk_glfw3_new_frame();
        int width,height;glfwGetWindowSize(window,&width,&height);const float sidebar=330,top=45,image_width=std::max(200.f,width-sidebar);
        if(nk_begin(ui,"Toolbar",nk_rect(0,0,float(width),top),NK_WINDOW_NO_SCROLLBAR)) {
            nk_layout_row_begin(ui,NK_STATIC,26,2);nk_layout_row_push(ui,110);if(nk_button_label(ui,"Save frame"))try{save_frame();}catch(const std::exception& e){message=e.what();}
            nk_layout_row_push(ui,float(width-150));nk_label(ui,status.c_str(),NK_TEXT_LEFT);nk_layout_row_end(ui);
        }nk_end(ui);
        for(int i=0;i<4;++i)pane(i,float(i%2)*image_width/2,top+float(i/2)*(height-top)/2,image_width/2,(height-top)/2);
        parameters(image_width,top,sidebar,height-top);dirty=false;glClearColor(.08f,.08f,.08f,1);glClear(GL_COLOR_BUFFER_BIT);nk_glfw3_render(NK_ANTI_ALIASING_ON);
        if(!pending_screenshot.empty()) {
            int w,h;glfwGetFramebufferSize(window,&w,&h);cv::Mat image(h,w,CV_8UC3);glPixelStorei(GL_PACK_ALIGNMENT,1);glReadBuffer(GL_BACK);
            glReadPixels(0,0,w,h,GL_RGB,GL_UNSIGNED_BYTE,image.data);cv::flip(image,image,0);png(pending_screenshot,image);pending_screenshot.clear();
        }
        glfwSwapBuffers(window);return true;
    }
};
VisionDebug::VisionDebug(AimPipeline& pipeline,VirtualCamera& camera,std::filesystem::path output,bool visible):impl(std::make_unique<Impl>(pipeline,camera,std::move(output),visible)){}
VisionDebug::~VisionDebug()=default;
bool VisionDebug::pump(){return impl->pump();}
void VisionDebug::show(){glfwShowWindow(impl->window);glfwFocusWindow(impl->window);}
void VisionDebug::update(const VirtualFrame& frame,const Json& result){impl->update(frame,result);}
void VisionDebug::save_frame(){impl->save_frame();}
}
