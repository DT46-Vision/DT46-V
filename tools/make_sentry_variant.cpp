#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>

int main(int argc,char** argv) {
    try {
        if(argc!=7)throw std::invalid_argument("Usage: dt46_sentry_variant INPUT.yaml OUTPUT.yaml q_xyz r_xyz_factor q_yaw q_r");
        std::ifstream input(argv[1]);if(!input)throw std::runtime_error("Cannot open input YAML");std::string text((std::istreambuf_iterator<char>(input)),{});
        const std::array names={"ekf_QR_q_xyz","ekf_QR_r_xyz_factor","ekf_QR_q_yaw","ekf_QR_q_r"};
        for(int i=0;i<4;++i){std::size_t used=0;const double number=std::stod(argv[i+3],&used);
            if(used!=std::string(argv[i+3]).size()||!std::isfinite(number)||number<0)throw std::invalid_argument("Noise parameters must be finite and nonnegative");
            const std::regex pattern(std::string("(^|\\n)([ \\t]*)")+names[i]+":([ \\t]*)[-+]?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][-+]?[0-9]+)?");
            std::smatch match;if(!std::regex_search(text,match,pattern))throw std::runtime_error(std::string("Parameter missing: ")+names[i]);
            std::ostringstream replacement;replacement<<match[1].str()<<match[2].str()<<names[i]<<':'<<match[3].str()<<std::setprecision(17)<<number;
            text.replace(match.position(),match.length(),replacement.str());}
        const std::filesystem::path destination=argv[2];if(destination.has_parent_path())std::filesystem::create_directories(destination.parent_path());
        std::ofstream output(destination);if(!output||!(output<<text))throw std::runtime_error("Cannot write output YAML");std::cout<<destination.string()<<'\n';return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
