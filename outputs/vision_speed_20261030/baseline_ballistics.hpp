// Audit-only snapshot of ee55828, never linked into runtime.
#include <algorithm>
#include <cmath>
struct Baseline {
 double bullet_speed=25,k_v2=.019;
 void build_ballistic_lut();
    static constexpr int LUT_DIST_BINS = 25;
    static constexpr int LUT_HEIGHT_BINS = 15;
    static constexpr double LUT_DIST_MIN = 0.1;
    static constexpr double LUT_DIST_MAX = 12.0;
    static constexpr double LUT_DIST_STEP = 0.5;
    static constexpr double LUT_HEIGHT_MIN = -3.5;
    static constexpr double LUT_HEIGHT_MAX = 3.5;
    static constexpr double LUT_HEIGHT_STEP = 0.5;
    double ballistic_lut_[LUT_DIST_BINS][LUT_HEIGHT_BINS];
    double tof_lut_[LUT_DIST_BINS][LUT_HEIGHT_BINS]; // 对应 (距离, 高度) 的子弹飞行时间 (s)

    double lut_lookup(double dist_h, double z) const;
    double lut_lookup_tof(double dist_h, double z) const;

};
void Baseline::build_ballistic_lut() {
    const double g = 9.81;
    const double v_init = bullet_speed > 0 ? bullet_speed : 28.0;
    const double sim_dt = 0.001; // 积分步长；比 5ms 细，保证飞行时间有 ms 级精度

    for (int i = 0; i < LUT_DIST_BINS; ++i) {
        for (int j = 0; j < LUT_HEIGHT_BINS; ++j) {
            double dist_h = LUT_DIST_MIN + i * LUT_DIST_STEP;
            double z_target = LUT_HEIGHT_MIN + j * LUT_HEIGHT_STEP;

            double pitch = std::atan2(z_target, dist_h);
            double tof = 0.0;

            for (int iter = 0; iter < 5; ++iter) {
                double sim_x = 0.0, sim_z = 0.0;
                double v_x = v_init * std::cos(pitch);
                double v_z = v_init * std::sin(pitch);
                double t = 0.0;

                while (sim_x < dist_h && t < 2.0) {
                    double v = std::hypot(v_x, v_z);
                    double a_x = -k_v2 * v * v_x;
                    double a_z = -g - k_v2 * v * v_z;

                    sim_x += v_x * sim_dt;
                    sim_z += v_z * sim_dt;
                    v_x += a_x * sim_dt;
                    v_z += a_z * sim_dt;
                    t += sim_dt;
                }
                tof = t;

                double z_error = z_target - sim_z;
                if (std::abs(z_error) < 0.005) break;
                pitch += z_error / std::max(dist_h, 0.1);
            }

            ballistic_lut_[i][j] = pitch;
            tof_lut_[i][j] = tof;
        }
    }
}

double Baseline::lut_lookup(double dist_h, double z) const {
    double di = std::clamp((dist_h - LUT_DIST_MIN) / LUT_DIST_STEP,0.0,double(LUT_DIST_BINS-1));
    double dj = std::clamp((z - LUT_HEIGHT_MIN) / LUT_HEIGHT_STEP,0.0,double(LUT_HEIGHT_BINS-1));

    int i0 = std::clamp(static_cast<int>(di), 0, LUT_DIST_BINS - 1);
    int j0 = std::clamp(static_cast<int>(dj), 0, LUT_HEIGHT_BINS - 1);
    int i1 = std::min(i0 + 1, LUT_DIST_BINS - 1);
    int j1 = std::min(j0 + 1, LUT_HEIGHT_BINS - 1);

    double fi = di - i0;
    double fj = dj - j0;

    return (1.0 - fi) * (1.0 - fj) * ballistic_lut_[i0][j0]
         + fi * (1.0 - fj) * ballistic_lut_[i1][j0]
         + (1.0 - fi) * fj * ballistic_lut_[i0][j1]
         + fi * fj * ballistic_lut_[i1][j1];
}

double Baseline::lut_lookup_tof(double dist_h, double z) const {
    double di = std::clamp((dist_h - LUT_DIST_MIN) / LUT_DIST_STEP,0.0,double(LUT_DIST_BINS-1));
    double dj = std::clamp((z - LUT_HEIGHT_MIN) / LUT_HEIGHT_STEP,0.0,double(LUT_HEIGHT_BINS-1));

    int i0 = std::clamp(static_cast<int>(di), 0, LUT_DIST_BINS - 1);
    int j0 = std::clamp(static_cast<int>(dj), 0, LUT_HEIGHT_BINS - 1);
    int i1 = std::min(i0 + 1, LUT_DIST_BINS - 1);
    int j1 = std::min(j0 + 1, LUT_HEIGHT_BINS - 1);

    double fi = di - i0;
    double fj = dj - j0;

    return (1.0 - fi) * (1.0 - fj) * tof_lut_[i0][j0]
         + fi * (1.0 - fj) * tof_lut_[i1][j0]
         + (1.0 - fi) * fj * tof_lut_[i0][j1]
         + fi * fj * tof_lut_[i1][j1];
}
