#pragma once
#include <cstdint>

#ifdef _WIN32
#define DT46_API extern "C" __declspec(dllexport)
#else
#define DT46_API extern "C" __attribute__((visibility("default")))
#endif

struct AimObservation {
    int id;
    double confidence;
    double position_camera_mm[3];
    double yaw_deg;
    double corners[8];
};

struct AimResult {
    int count;
    int light_count;
    int state;
    int tracked_id;
    int can_fire;
    int aim_valid;
    double yaw_command_deg;
    double pitch_command_deg;
    double yaw_error_deg;
    double pitch_error_deg;
    double position[3];
    double velocity[3];
    AimObservation observations[64];
};

DT46_API const char* dt46_error();
DT46_API int dt46_result_size();
DT46_API const char* dt46_snapshot(void* context);
DT46_API int dt46_debug_image(void* context, int kind, uint8_t* pixels, int capacity,
                              int* width, int* height, int* channels);
DT46_API void* dt46_create(const char* model, const char* parameters);
DT46_API void dt46_destroy(void* context);
DT46_API int dt46_process(void* context, const uint8_t* rgb, int width, int height,
                          double timestamp, const double* k, const double* d,
                          const double* optical_rotation, const double* muzzle_offset,
                          int target_color, double measured_speed, AimResult* result);
