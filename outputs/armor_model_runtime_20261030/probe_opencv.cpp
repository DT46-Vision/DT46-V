#include <opencv2/dnn.hpp>
#include <iostream>
int main(int argc,char** argv) {
    try {
        cv::setNumThreads(1);
        auto net=cv::dnn::readNetFromONNX(argv[1]);
        int shape[]={1,3,128,128};
        cv::Mat image(4,shape,CV_32F,cv::Scalar(.25));
        cv::Mat camera(1,9,CV_32F,cv::Scalar(0));
        camera.at<float>(0,0)=camera.at<float>(0,1)=1;
        camera.at<float>(0,2)=camera.at<float>(0,3)=.5;
        net.setInput(image,"image");net.setInput(camera,"camera");
        std::vector<cv::Mat> outputs;
        const std::vector<cv::String> names={"armor_logit","sticker_logits","team_logits","position_camera_m","rotation_6d"};
        net.forward(outputs,names);
        for(size_t i=0;i<outputs.size();++i)std::cout<<names[i]<<": "<<outputs[i]<<'\n';
        return 0;
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
