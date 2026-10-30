#include "armor_model.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace dt46 {
namespace {
using Clock=std::chrono::steady_clock;
double milliseconds(Clock::time_point start){return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
double parameter(const cv::FileNode& node,const char* name,double fallback){return node[name].empty()?fallback:double(node[name]);}
int number(const std::string& name) {
    if(name=="engineer_2")return 2;
    if(name=="infantry_3")return 3;
    if(name=="infantry_4")return 4;
    if(name=="sentry")return 7;
    if(name=="base_small"||name=="outpost")return -1;
    throw std::invalid_argument("Unsupported model sticker: "+name);
}
std::pair<int,double> classification(const cv::Mat& logits) {
    const float* values=logits.ptr<float>();
    const int best=int(std::max_element(values,values+logits.total())-values);
    double sum=0;for(size_t i=0;i<logits.total();++i)sum+=std::exp(double(values[i]-values[best]));
    return {best,1/sum};
}
struct Light {cv::Point2f center,axis;float height,angle;};
struct Candidate {int left,right;cv::Rect roi;std::array<cv::Point2f,4> corners;double quality;};
}

ArmorModel::ArmorModel(const std::string& path,const cv::FileNode& p) {
    auto metadata=std::filesystem::path(path);metadata.replace_extension(".json");
    cv::FileStorage info(metadata.string(),cv::FileStorage::READ);
    if(!info.isOpened())throw std::invalid_argument("Missing armor model metadata: "+metadata.string());
    if(int(info["format_version"])!=1||int(info["image_size"])!=128||int(info["batch_size"])!=1||
       std::string(info["position_units"])!="metres"||std::string(info["camera_axes"])!="right/down/forward"||
       std::string(info["board_axes"])!="right/up/outward"||
       std::string(info["rotation_6d_order"])!="first column of R_camera_from_board, then second column")
        throw std::invalid_argument("Incompatible armor model metadata; export with --fixed-batch");
    std::vector<std::string> teams;info["teams"]>>teams;
    if(teams!=std::vector<std::string>{"red","blue"})throw std::invalid_argument("Incompatible model team order");
    info["classes"]>>classes_;
    if(classes_.empty())throw std::invalid_argument("Empty model sticker catalog");
    for(const auto& name:classes_){
        const auto family=std::string(info["catalog"][name]["target_family"]);
        if(family!=(number(name)>0?"vehicle":"structure"))throw std::invalid_argument("Inconsistent model target family");
        families_.push_back(family);
    }
    net_=cv::dnn::readNetFromONNX(path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    display=parameter(p,"display_mode",0)!=0;
    binary_val_=int(parameter(p,"binary_val",120));
    min_area_=parameter(p,"light_area_min",5);min_ratio_=parameter(p,"light_h_w_ratio",2);
    max_angle_=parameter(p,"light_angle_max",75);min_color_difference_=parameter(p,"light_color_difference",20);
    height_ratio_=parameter(p,"height_rate_tol",2.5);
    spacing_min_=parameter(p,"height_multiplier_min",.5);spacing_max_=parameter(p,"height_multiplier_max",4);
    crop_height_=parameter(p,"crop_height",2.2);crop_margin_=parameter(p,"crop_margin",1.35);
    presence_threshold_=parameter(p,"presence_threshold",.5);
    roi_crop_=parameter(p,"roi_crop",0)!=0;roi_scale_=parameter(p,"roi_scale",.4);
    // Warm up and validate tensor sizes before replacing a running pipeline.
    infer(cv::Mat(128,128,CV_8UC3,cv::Scalar(20,20,20)),cv::Matx33d(128,0,64,0,128,64,0,0,1),{});
}

cv::Matx33d ArmorModel::decode_rotation(const float* axes) {
    cv::Vec3d x(axes[0],axes[1],axes[2]),y(axes[3],axes[4],axes[5]);
    const double nx=cv::norm(x);if(!std::isfinite(nx)||nx<1e-6)throw std::domain_error("Invalid model rotation axis");
    x/=nx;y-=x*x.dot(y);
    const double ny=cv::norm(y);if(!std::isfinite(ny)||ny<1e-6)throw std::domain_error("Collinear model rotation axes");
    y/=ny;const auto n=x.cross(y);
    return {x[0],y[0],n[0],x[1],y[1],n[1],x[2],y[2],n[2]};
}

cv::Mat ArmorModel::crop(const cv::Mat& rgb,const cv::Rect& roi,const cv::Matx33d& k,cv::Matx33d& crop_k) {
    if(roi.empty()||(roi&cv::Rect(0,0,rgb.cols,rgb.rows))!=roi)throw std::invalid_argument("Invalid model crop");
    const double scale=128./std::max(roi.width,roi.height);
    const int w=std::max(1,int(std::lround(roi.width*scale))),h=std::max(1,int(std::lround(roi.height*scale)));
    const int px=(128-w)/2,py=(128-h)/2;
    cv::Mat image(128,128,CV_8UC3,cv::Scalar(20,20,20));
    cv::resize(rgb(roi),image(cv::Rect(px,py,w,h)),{w,h},0,0,cv::INTER_LINEAR);
    const double sx=double(w)/roi.width,sy=double(h)/roi.height;
    // Match the dataset's pixel-centre convention; D remains unchanged.
    crop_k=cv::Matx33d(sx,0,px-sx*roi.x+.5*sx-.5,0,sy,py-sy*roi.y+.5*sy-.5,0,0,1)*k;
    return image;
}

ModelObservation ArmorModel::infer(const cv::Mat& image,const cv::Matx33d& k,const cv::Vec<double,5>& d) {
    if(image.type()!=CV_8UC3||image.size()!=cv::Size(128,128)||!cv::checkRange(cv::Mat(k))||
       !cv::checkRange(cv::Mat(d))||k(0,0)<=0||k(1,1)<=0||k(0,1)!=0||k(1,0)!=0||
       k(2,0)!=0||k(2,1)!=0||k(2,2)!=1)throw std::invalid_argument("Invalid model image or calibration");
    cv::Mat camera(1,9,CV_32F);
    camera.at<float>(0,0)=std::log(float(k(0,0))/128);camera.at<float>(0,1)=std::log(float(k(1,1))/128);
    camera.at<float>(0,2)=float(k(0,2))/128;camera.at<float>(0,3)=float(k(1,2))/128;
    for(int i=0;i<5;++i)camera.at<float>(0,4+i)=float(d[i]);
    net_.setInput(cv::dnn::blobFromImage(image,1./255,{}, {},false,false),"image");
    net_.setInput(camera,"camera");
    std::vector<cv::Mat> output;
    net_.forward(output,std::vector<cv::String>{"armor_logit","sticker_logits","team_logits","position_camera_m","rotation_6d"});
    const size_t sizes[]={1,classes_.size(),2,3,6};
    for(size_t i=0;i<5;++i){
        if(output[i].type()!=CV_32F||output[i].total()!=sizes[i])throw std::runtime_error("Invalid armor model output shape");
        if(!cv::checkRange(output[i]))throw std::domain_error("Nonfinite armor model output");
    }
    ModelObservation result;
    result.presence=1/(1+std::exp(-double(output[0].ptr<float>()[0])));
    const auto [sticker,sticker_score]=classification(output[1]);
    const auto [team,team_score]=classification(output[2]);
    result.sticker=classes_[sticker];result.vehicle=families_[sticker]=="vehicle";
    result.robot_number=number(result.sticker);result.team=team;
    result.sticker_score=sticker_score;result.team_score=team_score;
    result.armor_id=result.vehicle?(result.robot_number==7?5:result.robot_number-1)+(team==0?6:0):-1;
    for(int i=0;i<3;++i)result.position[i]=output[3].ptr<float>()[i];
    result.rotation=decode_rotation(output[4].ptr<float>());
    if(result.position[2]<=0)throw std::domain_error("Nonpositive model depth");
    return result;
}

std::vector<ModelObservation> ArmorModel::detect(const cv::Mat& rgb,const cv::Matx33d& k,const cv::Vec<double,5>& d,int color) {
    const auto start=Clock::now();for(auto& image:images)image.release();invalid_pose_count=0;
    cv::Rect search(0,0,rgb.cols,rgb.rows);
    if(roi_crop_){search.width=std::max(1,int(rgb.cols*roi_scale_));search.height=std::max(1,int(rgb.rows*roi_scale_));search.x=(rgb.cols-search.width)/2;search.y=(rgb.rows-search.height)/2;}
    const cv::Mat raw=rgb(search);
    cv::Mat mask(raw.size(),CV_8U);
    for(int y=0;y<raw.rows;++y)for(int x=0;x<raw.cols;++x){
        const auto p=raw.at<cv::Vec3b>(y,x);
        // Chroma only proposes lights. Team/number come exclusively from the model.
        mask.at<uchar>(y,x)=std::max(p[0],p[2])>=binary_val_&&std::abs(int(p[0])-int(p[2]))>=min_color_difference_?255:0;
    }
    std::vector<std::vector<cv::Point>> contours;cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    std::vector<Light> lights;
    for(const auto& contour:contours){
        if(cv::contourArea(contour)<min_area_)continue;
        auto rect=cv::minAreaRect(contour);float angle=rect.angle;
        if(rect.size.width>rect.size.height){std::swap(rect.size.width,rect.size.height);angle-=90;}
        if(rect.size.height<2||rect.size.height/std::max(.5f,rect.size.width)<min_ratio_||std::abs(angle)>max_angle_)continue;
        const float radians=angle*float(CV_PI/180);
        lights.push_back({rect.center+cv::Point2f(float(search.x),float(search.y)),{-std::sin(radians),std::cos(radians)},rect.size.height,angle});
    }
    // Bound work on highly textured frames, keeping the largest visible lights.
    std::stable_sort(lights.begin(),lights.end(),[](const Light& a,const Light& b){return a.height>b.height;});
    if(lights.size()>64)lights.resize(64);
    light_count=int(lights.size());std::vector<Candidate> candidates;
    for(int i=0;i<light_count;++i)for(int j=i+1;j<light_count;++j){
        const auto& a=lights[i];const auto& b=lights[j];const auto delta=b.center-a.center;
        const double distance=cv::norm(delta),height=(a.height+b.height)*.5;
        if(std::max(a.height,b.height)/std::min(a.height,b.height)>height_ratio_||distance<height*spacing_min_||distance>height*spacing_max_)continue;
        const double tilt=std::abs(a.angle-b.angle);
        if(tilt>25||std::abs(delta.dot(a.axis))/distance>.6||std::abs(delta.dot(b.axis))/distance>.6)continue;
        Candidate c{i,j,{}, {a.center-a.axis*a.height*.5f,a.center+a.axis*a.height*.5f,b.center-b.axis*b.height*.5f,b.center+b.axis*b.height*.5f},tilt/25+std::abs(std::log(a.height/b.height))};
        std::vector<cv::Point2f> bounds;
        for(const auto* light:{&a,&b})for(int sign:{-1,1})bounds.push_back(light->center+light->axis*float(sign*light->height*.5*crop_height_));
        float x0=bounds[0].x,x1=x0,y0=bounds[0].y,y1=y0;
        for(const auto& p:bounds){x0=std::min(x0,p.x);x1=std::max(x1,p.x);y0=std::min(y0,p.y);y1=std::max(y1,p.y);}
        const double mx=(x1-x0)*(crop_margin_-1)*.5,my=(y1-y0)*(crop_margin_-1)*.5;
        const int left=std::clamp(int(std::floor(x0-mx)),0,rgb.cols),right=std::clamp(int(std::ceil(x1+mx)),0,rgb.cols);
        const int top=std::clamp(int(std::floor(y0-my)),0,rgb.rows),bottom=std::clamp(int(std::ceil(y1+my)),0,rgb.rows);
        c.roi={left,top,right-left,bottom-top};if(c.roi.area()>0)candidates.push_back(c);
    }
    std::stable_sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){return a.quality<b.quality;});
    if(candidates.size()>64)candidates.resize(64);
    candidate_count=int(candidates.size());candidate_ms=milliseconds(start);const auto model_start=Clock::now();
    std::vector<std::pair<ModelObservation,size_t>> accepted;
    for(size_t i=0;i<candidates.size();++i){
        cv::Matx33d crop_k;auto input=crop(rgb,candidates[i].roi,k,crop_k);
        ModelObservation observation;
        try{observation=infer(input,crop_k,d);}catch(const std::domain_error&){++invalid_pose_count;continue;}
        if(display)images[3]=input;
        if(observation.presence<presence_threshold_||(color!=2&&observation.team!=color))continue;
        observation.roi=candidates[i].roi;observation.corners=candidates[i].corners;
        accepted.emplace_back(std::move(observation),i);
    }
    std::stable_sort(accepted.begin(),accepted.end(),[](const auto& a,const auto& b){return a.first.presence>b.first.presence;});
    std::array<bool,64> used{};std::vector<ModelObservation> result;
    for(auto& [observation,index]:accepted){
        const auto& c=candidates[index];if(used[c.left]||used[c.right])continue;
        bool duplicate=false;for(const auto& previous:result){const double area=(previous.roi&c.roi).area();if(area/(previous.roi.area()+c.roi.area()-area)>.5){duplicate=true;break;}}
        if(duplicate)continue;
        used[c.left]=used[c.right]=true;result.push_back(std::move(observation));
    }
    model_ms=milliseconds(model_start);
    if(display){
        images[0]=rgb.clone();images[1]=raw;images[2]=mask;images[4]=rgb;
        cv::cvtColor(raw,images[5],cv::COLOR_RGB2GRAY);
        for(const auto& o:result)cv::rectangle(images[0],o.roi,{255,220,0},2);
    }
    return result;
}
}
