#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include "modules/media/calibration.hpp"

int main(int argc,char** argv) {
    try {
        std::filesystem::path images,output;cv::Size board(8,6);double square=.025;std::string name="camera";
        for(int i=1;i<argc;++i){const std::string arg=argv[i];const auto value=[&]{if(++i>=argc)throw std::invalid_argument("Missing value: "+arg);return std::string(argv[i]);};
            if(arg=="--images")images=value();else if(arg=="--output")output=value();else if(arg=="--square")square=std::stod(value());
            else if(arg=="--columns")board.width=std::stoi(value());else if(arg=="--rows")board.height=std::stoi(value());else if(arg=="--name")name=value();
            else if(arg=="--help"){std::cout<<"dt46_calibrate --images DIRECTORY --output camera_info.yaml [--columns 8 --rows 6 --square 0.025 --name camera]\nUse diverse chessboard poses; columns/rows count INNER corners, square is metres.\n";return 0;}
            else throw std::invalid_argument("Unknown argument: "+arg);}
        if(images.empty()||output.empty())throw std::invalid_argument("--images and --output are required");
        dt46::chessboard_points(board,square);std::vector<std::filesystem::path> paths;
        for(const auto& entry:std::filesystem::directory_iterator(images)){const auto ext=entry.path().extension().string();if(entry.is_regular_file()&&(ext==".png"||ext==".jpg"||ext==".jpeg"||ext==".bmp"||ext==".tif"))paths.push_back(entry.path());}
        std::sort(paths.begin(),paths.end());cv::Size image_size;std::vector<std::vector<cv::Point2f>> observations;nlohmann::json report={{"accepted",nlohmann::json::array()},{"rejected",nlohmann::json::array()}};
        for(const auto& path:paths){const auto image=cv::imread(path.string(),cv::IMREAD_GRAYSCALE);if(image.empty())throw std::runtime_error("Cannot read "+path.string());
            if(image_size.empty())image_size=image.size();if(image.size()!=image_size)throw std::invalid_argument("Mixed image resolutions");
            std::vector<cv::Point2f> corners;if(cv::findChessboardCornersSB(image,board,corners,cv::CALIB_CB_NORMALIZE_IMAGE|cv::CALIB_CB_EXHAUSTIVE|cv::CALIB_CB_ACCURACY)){
                observations.push_back(std::move(corners));report["accepted"].push_back(path.filename().string());}else report["rejected"].push_back(path.filename().string());}
        const auto calibration=dt46::calibrate(observations,image_size,board,square);dt46::save_calibration(output,calibration,name);
        report["rms_pixels"]=calibration.rms;report["per_view_rms_pixels"]=calibration.per_view_rms;report["square_metres"]=square;report["inner_corners"]={board.width,board.height};
        std::ofstream summary(output.string()+".json");if(!summary||!(summary<<report.dump(2)<<'\n'))throw std::runtime_error("Cannot write calibration report");
        std::cout<<report.dump(2)<<'\n';return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
