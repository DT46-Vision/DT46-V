#include "rm_detector/classifier.hpp"
#include <iostream>

namespace DT46_VISION {

    NumberClassifier::NumberClassifier(const std::string& onnx_path,
                                       cv::Size input_size)
        : input_size_(input_size)
    {
        net_ = cv::dnn::readNetFromONNX(onnx_path);
        if (net_.empty()) {
            throw std::runtime_error("NumberClassifier: failed to load ONNX model: " + onnx_path);
        }

        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        try {
    // 1. 指定使用 CUDA 后端
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    
    // 2. 关键修改：强制开启 FP16 半精度推理（Orin 算力核心）
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    
    // 3. 预热过程 (Warm-up) 保持不变
    cv::Mat dummy(20, 28, CV_8UC1, cv::Scalar(0));
    cv::Mat blob = cv::dnn::blobFromImage(dummy, 1.0/255.0, input_size_, cv::Scalar(), false, false, CV_32F);
    net_.setInput(blob);
    net_.forward();
    
    // 可选：加个打印，确认预热成功
    std::cout << "[Detector] ✅ CUDA FP16 Backend initialized successfully!" << std::endl;

} catch (const cv::Exception& e) {
    // 4. 关键修改：千万不要悄悄降级！一定要把报错原因打印出来
    std::cerr << "[Detector] ❌ CUDA execution failed: " << e.what() << std::endl;
    std::cerr << "[Detector] ⚠️ Falling back to CPU. Performance will drop significantly!" << std::endl;
    
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

        bin_f_.create(input_size_, CV_8UC1);
        blob_.create(1, 1, CV_32F);
    }

    NumberClassifier::Result NumberClassifier::classify(const cv::Mat& armor_img)
    {
        Result out;
        if (armor_img.empty()) return out;

        cv::resize(armor_img, bin_f_, input_size_, 0, 0, cv::INTER_AREA);

        blob_ = cv::dnn::blobFromImage(bin_f_, 1.0 / 255.0, input_size_, cv::Scalar(), false, false, CV_32F);

        net_.setInput(blob_);
        cv::Mat logits = net_.forward();

        cv::Point classIdPoint;
        double confidence;
        cv::minMaxLoc(logits, nullptr, &confidence, nullptr, &classIdPoint);

        out.class_id = classIdPoint.x;
        out.confidence = static_cast<float>(confidence);

        return out;
    }

} // namespace DT46_VISION
