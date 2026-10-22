"""Reproduce four logic counterexamples from the reviewed commit, not integration tests."""

from pathlib import Path
import shutil
import subprocess
import tempfile

SOURCE_COMMIT = '3c41f0b02a05bf06ad28ca8440bc8099545498ab'
root = Path(__file__).resolve().parents[3]


def read_source(path: str) -> str:
    return subprocess.run(
        ['git', '-C', str(root), 'show', f'{SOURCE_COMMIT}:{path}'],
        check=True, capture_output=True, encoding='utf-8',
    ).stdout


source = read_source('src/rm_tracker/src/tracker.cpp')
detector = read_source('src/rm_detector/src/detector.cpp')
header = read_source('src/rm_tracker/include/rm_tracker/tracker.hpp')


def function(text: str, signature: str) -> str:
    start = text.index(signature)
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


prefix = r'''
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <tuple>
#include <vector>
#include <cassert>
constexpr double CV_PI = 3.14159265358979323846;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
constexpr double DEG2RAD = CV_PI / 180.0, RAD2DEG = 180.0 / CV_PI;
namespace Eigen {
template<class T, int N, int M> struct Matrix {
    std::array<double, N> data{};
    Matrix() = default;
    template<class... Args> Matrix(Args... args): data{double(args)...} {}
    double& operator()(int i) { return data[i]; }
    double operator()(int i) const { return data[i]; }
    Matrix operator+(const Matrix& other) const {
        Matrix result; for(int i=0;i<N;++i) result(i)=data[i]+other(i); return result;
    }
    Matrix operator*(double s) const {
        Matrix result; for(int i=0;i<N;++i) result(i)=data[i]*s; return result;
    }
};
using Vector3d = Matrix<double,3,1>;
using Vector4d = Matrix<double,4,1>;
}
struct Armor {
    int id; Eigen::Vector3d pos; double yaw, dist, target_to_armor_dist=0, angle_diff=0;
    Armor(int id_, double x, double y, double z, double a):
      id(id_),pos(x,y,z),yaw(a),dist(std::hypot(x,y,z)) {}
};
// Measurement sink: captures exactly what apply_armor_measurement sends.
// It deliberately does not emulate an EKF update or claim end-to-end results.
struct MeasurementSink {
    Eigen::Matrix<double,9,1> state;
    Eigen::Vector4d observed;
    bool update(const Eigen::Vector4d& z) { observed=z; return true; }
    Eigen::Matrix<double,9,1> getState() const { return state; }
};
class Tracker {
public:
    struct XySample { double t,x,y; };
    struct Appearance { int id; double armor_width, armor_height; };
    std::deque<XySample> obs_center_hist_;
    double move_clock_=0, spin_aim_mode_clock_=0;
    bool spin_=true, spin_aim_translate_=false;
    static constexpr double kAimEnterTrans=.45, kAimLeaveTrans=.30;
    double yaw_tolerance_deg=5, pitch_tolerance_deg=2, shootable_dist=10;
    double bullet_speed=32, k_v2=.019, system_delay=.1, controller_delay_=.02;
    double last_plate_yaw_=0;
    int tracked_id=0;
    Eigen::Matrix<double,9,1> target_state_;
    Eigen::Matrix<double,6,1> plate_;
    Eigen::Vector3d lead_vel_;
    MeasurementSink ekf_;
    std::vector<Appearance> robot_list_;
    static constexpr int LUT_DIST_BINS=25,LUT_HEIGHT_BINS=15;
    static constexpr double LUT_DIST_MIN=.1,LUT_DIST_MAX=12,LUT_DIST_STEP=.5;
    static constexpr double LUT_HEIGHT_MIN=-3.5,LUT_HEIGHT_STEP=.5;
    double ballistic_lut_[LUT_DIST_BINS][LUT_HEIGHT_BINS];
    double tof_lut_[LUT_DIST_BINS][LUT_HEIGHT_BINS];
    double orientation_to_yaw(double,double) const;
    double disambiguate_armor_yaw(double,const Eigen::Vector3d&,double,double,double) const;
    bool center_motion_is_slow() const;
    void update_spin_aim_mode(double);
    bool apply_armor_measurement(const Armor&);
    bool yaw_in_armor_edge_window(double,double,double,double,double,double,double*,double*) const;
    std::tuple<double,double,bool> can_fire(const Armor&, std::tuple<double,double,bool>);
    void build_ballistic_lut();
    double lut_lookup_tof(double,double) const;
    Armor stationary_aim();
};
struct Point { double x,y; };
struct Light { Point up,down; int cx,cy; };
'''

parts = [prefix]
for signature in ['inline double normalize_angle(', 'inline double shortest_angular_distance(']:
    parts.append(function(header, signature))
for signature in [
    'double Tracker::orientation_to_yaw(', 'double Tracker::disambiguate_armor_yaw(',
    'bool Tracker::center_motion_is_slow(', 'void Tracker::update_spin_aim_mode(',
    'bool Tracker::apply_armor_measurement(', 'bool Tracker::yaw_in_armor_edge_window(',
    'std::tuple<double, double, bool> Tracker::can_fire(',
    'void Tracker::build_ballistic_lut(', 'double Tracker::lut_lookup_tof(',
]:
    parts.append(function(source, signature))

fallback_start = source.index('        const Eigen::Vector3d plate_pos(', source.index('Tracker::track('))
fallback_end = source.index('\n    }\n', fallback_start)
parts.append('Armor Tracker::stationary_aim() { Armor target_cam(0,0,0,0,0);\n' +
             source[fallback_start:fallback_end] + '\nreturn target_cam;\n}')

slope_start = detector.index('        const double slope_limit =')
slope_end = detector.index('        for (size_t i =', slope_start)
parts.append('int main() {\nstd::cout << std::fixed << std::setprecision(6);\n'
             'struct { int light_angle_max=35; } params;\n' + detector[slope_start:slope_end])
parts.append(r'''
    // Light.up uses the bottom midpoint; down uses the top midpoint, as
    // detector.cpp:212-218 and OpenCV RotatedRect::points ordering specify.
    Light a{{100,106},{101,94},100,100};
    Light b{{128,106},{127,94},127,100};
    Light b_same{{128,106},{129,94},128,100};
    const double raw=std::abs(light_tilt_deg(a)-light_tilt_deg(b));
    const double wrapped=std::abs(shortest_angular_distance(light_tilt_deg(a)*DEG2RAD,
                                                         light_tilt_deg(b)*DEG2RAD))*RAD2DEG;
    std::cout << "LIGHT raw_difference_deg=" << raw << " geometric_difference_deg=" << wrapped
              << " pair_opposite=" << slopes_match(a,b) << " pair_same=" << slopes_match(a,b_same) << '\n';
    assert(!slopes_match(a,b) && slopes_match(a,b_same) && wrapped<35);

    Tracker t;
    t.obs_center_hist_={{0,5,0},{.5,5,.4}};
    t.move_clock_=.5;
    bool before=t.center_motion_is_slow();
    for(int i=0;i<200;++i) { t.move_clock_+=.01; t.update_spin_aim_mode(.01); }
    std::cout << "HISTORY age_s=" << t.move_clock_-t.obs_center_hist_.back().t
              << " slow_before=" << before << " slow_after=" << t.center_motion_is_slow()
              << " translating_aim=" << t.spin_aim_translate_ << '\n';
    assert(!t.center_motion_is_slow() && t.spin_aim_translate_);

    const double ref=30*DEG2RAD, r=.24;
    Armor observation(0,5,0,0,-30*DEG2RAD);
    const double corrected=t.disambiguate_armor_yaw(observation.yaw,observation.pos,ref,
                                                    5+r*std::cos(ref),r*std::sin(ref));
    t.target_state_(6)=ref;
    t.ekf_.state=t.target_state_;
    t.apply_armor_measurement(observation);
    std::cout << "YAW corrected_deg=" << corrected*RAD2DEG
              << " gate_residual_deg=" << std::abs(shortest_angular_distance(ref,corrected))*RAD2DEG
              << " actual_observation_deg=" << t.ekf_.observed(3)*RAD2DEG << '\n';
    assert(std::abs(corrected-ref)<1e-6 && std::abs(t.ekf_.observed(3)+ref)<1e-6);

    Tracker spin_tracker;
    spin_tracker.build_ballistic_lut();
    spin_tracker.plate_(0)=5; spin_tracker.target_state_(8)=r; spin_tracker.last_plate_yaw_=0;
    const Armor aimed=spin_tracker.stationary_aim();
    const double arrival=spin_tracker.lut_lookup_tof(5,0)+spin_tracker.system_delay;
    const double phase=4*arrival;
    const double arrival_err=std::abs(std::remainder(phase,CV_PI/2));
    const auto cmd=spin_tracker.can_fire(aimed,{0,0,true});
    std::cout << "SPIN arrival_s=" << arrival << " rotation_deg=" << phase*RAD2DEG
              << " future_nearest_plate_err_deg=" << arrival_err*RAD2DEG
              << " future_arc_error_m=" << r*arrival_err
              << " allowed_half_width_m=" << .135/2-.0085
              << " current_can_fire=" << std::get<2>(cmd) << '\n';
    assert(std::get<2>(cmd) && r*arrival_err > .135/2-.0085);
    std::cout << "All four counterexamples reproduced; not a ROS/camera/EKF integration test.\n";
}
''')

compiler = shutil.which('g++')
if compiler is None:
    raise SystemExit('g++ with C++17 support must be available on PATH.')

with tempfile.TemporaryDirectory(prefix='dt46-review-') as directory:
    out = Path(directory)
    cpp = out / 'review_logic.cpp'
    cpp.write_text('\n\n'.join(parts), encoding='utf-8')
    exe = out / 'review_logic.exe'
    subprocess.run([compiler, '-std=c++17', '-O2', str(cpp), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    print(result.stdout, end='')
