#include "rm_detector/detector.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

DT46_VISION::Params parameters() {
    return {5, 2., -35, 35, 1., 1., 1.8, 1., 3., 2.22, .7, false, .4};
}

DT46_VISION::Light shifted(DT46_VISION::Light light, float x, float y = 0) {
    return {light.up + cv::Point2f(x, y), light.down + cv::Point2f(x, y), light.color};
}
}

int main(int argc, char** argv) {
    try {
        require(argc >= 3, "Expected saved frame and classifier model");
        const auto input = cv::imread(argv[1]);
        require(!input.empty(), "Missing detector regression frame");
        cv::Mat fractional(160,80,CV_8UC3,cv::Scalar(10,10,10));
        for(int y=0;y<fractional.rows;++y){
            const double coverage=std::max(0.,std::min(y+.5,120.7)-std::max(y-.5,80.2));
            for(int x=35;x<=41;++x)fractional.at<cv::Vec3b>(y,x)=cv::Vec3b(
                uchar(10+245*coverage),uchar(10+210*coverage),10);
        }
        DT46_VISION::ArmorDetector edge_detector(1,false,120,parameters());
        const auto edge_lights=edge_detector.find_lights(edge_detector.process(fractional));
        require(edge_lights.size()==1 && std::abs(edge_lights[0].up.y-120.7)<.35 &&
                std::abs(edge_lights[0].down.y-80.2)<.35,
                "Light endpoints retained pixel-center shortening on fractional edges");
        DT46_VISION::ArmorDetector detector(1, false, 120, parameters());
        detector.set_classifier(std::make_shared<DT46_VISION::NumberClassifier>(argv[2]));
        auto lights = detector.find_lights(detector.process(input));
        std::sort(lights.begin(), lights.end(), [](const auto& a, const auto& b) { return a.cx < b.cx; });
        require(lights.size() == 4, "Saved frame must retain the interfering digit contour");
        const auto expected = detector.is_close(lights[1], lights[3]);
        require(expected.first.class_id == 1, "Real light pair must still classify as 3");
        const auto detected = detector.is_armor(lights);
        std::cout << "saved_frame lights=" << lights.size() << " armors=" << detected.size() << '\n';
        if (argc == 4 && std::string(argv[3]) == "--benchmark") {
            const auto measure = [](const char* name, const auto& run) {
                for (int i = 0; i < 20; ++i) run();
                std::vector<double> samples;
                for (int i = 0; i < 300; ++i) {
                    const auto start = std::chrono::steady_clock::now();
                    run();
                    samples.push_back(std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - start).count());
                }
                std::sort(samples.begin(), samples.end());
                std::cout << name << "_ms median=" << samples[150] << " p95=" << samples[285] << '\n';
            };
            measure("detect", [&] { detector.detect_armors(input); });
            const std::vector<DT46_VISION::Light> pair{lights[1], lights[3]};
            require(detector.is_armor(pair).size() == 1, "Unobstructed comparison must detect one armor");
            measure("unobstructed_pair", [&] { detector.is_armor(pair); });
            return 0;
        }
        require(detected.size() == 1 && detected[0].armor_id == 2 &&
                    detected[0].light1_up == lights[1].up && detected[0].light2_up == lights[3].up,
                "Digit contour blocked or replaced the real armor pair");

        // Accepted spans cannot share lights or pair their interior with a later plate.
        const auto inner = shifted(lights[1], 1);
        const auto later = shifted(lights[3], 1);
        require(detector.is_close(inner, later).first.class_id == 1,
                "Crossing counterexample must pass classification in isolation");
        const std::vector<DT46_VISION::Light> crossing{lights[1], inner, lights[3], later};
        require(detector.is_armor(crossing).size() == 1, "Accepted armor allowed a crossing pair");

        // Two separate plates must survive interval ownership, including ROI offsets.
        cv::Mat two_plates;
        cv::hconcat(input, input, two_plates);
        require(detector.detect_armors(two_plates).size() == 2, "Adjacent plates were merged or lost");
        cv::vconcat(input, input, two_plates);
        require(detector.detect_armors(two_plates).size() == 2,
                "Disjoint plates with overlapping x intervals were lost");
        detector.params.light_red_ratio = detector.params.light_blue_ratio = 1.3;
        require(detector.detect_armors(input).size() == 1 && detector.lights.size() == 3,
                "Color contrast did not remove the white digit while preserving real lights");
        cv::Mat white;
        cv::cvtColor(input, white, cv::COLOR_BGR2GRAY);
        cv::cvtColor(white, white, cv::COLOR_GRAY2BGR);
        require(detector.detect_armors(white).empty() && detector.lights.empty(),
                "Achromatic white contours passed the color gate");
        detector.params = parameters();
        cv::Mat rgb_input;
        cv::cvtColor(input, rgb_input, cv::COLOR_BGR2RGB);
        const auto rgb_detected = detector.detect_armors(rgb_input, true);
        require(rgb_detected.size() == detected.size() && rgb_detected[0].armor_id == detected[0].armor_id &&
                cv::norm(rgb_detected[0].light1_up-detected[0].light1_up)<1e-6 &&
                rgb_detected[0].res.confidence == detected[0].res.confidence,
                "RGB path changed detection or classification");
        cv::Mat padded(input.rows * 2, input.cols * 2, CV_8UC3, cv::Scalar());
        input.copyTo(padded(cv::Rect(input.cols / 2, input.rows / 2, input.cols, input.rows)));
        detector.params.roi_crop = true;
        detector.params.roi_scale = .5;
        const auto cropped = detector.detect_armors(padded);
        require(cropped.size() == 1 && cv::norm(cropped[0].light1_up - detected[0].light1_up -
                    cv::Point2f(input.cols / 2, input.rows / 2)) < .001,
                "ROI pairing did not restore full-frame coordinates");
        detector.params = parameters();
        detector.process(input);
        std::reverse(lights.begin(), lights.end());
        require(detector.is_armor(lights).size() == 1, "Contour order changed pairing");
        require(detector.is_armor({lights[0]}).empty() && detector.armors.empty(),
                "Single light retained the previous armor cache");
        require(detector.is_armor({}).empty() && detector.armors.empty(), "Empty lights retained armor cache");

        detector.params.height_multiplier_max = 1.5;
        require(detector.detect_armors(input).empty(), "Width gate was relaxed to recover detection");
        detector.params = parameters();
        detector.update_detect_color(0);
        require(detector.detect_armors(input).empty(), "Blue armor passed red-only detection");
        detector.update_detect_color(2);
        require(detector.detect_armors(input).size() == 1, "Both-color mode lost the saved armor");
        const auto blank = cv::Mat::zeros(input.size(), CV_8UC3);
        require(detector.detect_armors(blank).empty() && detector.armors.empty(), "Blank frame retained armor");
        require(detector.is_close(lights[2], lights[0]).first.class_id == 3 &&
                    detector.is_armor({lights[2], lights[0]}).empty(),
                "Blank classifier crop must be rejected despite valid pair geometry");
        detector.detect_armors(input);
        require(detector.detect_armors({}).empty() && detector.armors.empty() && detector.lights.empty(),
                "Empty image retained detection state");
        std::cout << "PASS: digit obstruction, pair ownership, order, geometry, colors and stale state\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
