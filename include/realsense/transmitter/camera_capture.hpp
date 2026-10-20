#pragma once

#include "common/data_buffer.hpp"
#include "realsense/color_frame.hpp"
#include "realsense/depth_frame.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace rs2 { class pipeline; }

namespace kist {

// Optional RealSense spatial (edge-preserving) filter on depth. Disabled
// unless enabled; params match librealsense's RS2_OPTION_FILTER_*.
struct SpatialFilterConfig {
    bool  enabled      = false;
    int   magnitude    = 2;      // filter passes, 1..5
    float smooth_alpha = 0.5f;   // neighbor weight, 0..1
    float smooth_delta = 20.0f;  // depth-discontinuity tolerance (mm)
};

// Exposure of the sensor that produces the COLOR stream. Default: device
// auto-exposure (AE). Fixing it (auto_exposure=false) stops AE hunting and
// mains-flicker pulsing — units are the sensor's native ones (D455 RGB:
// ticks of 100 µs, default 156 ≈ 15.6 ms; D405 color rides the stereo
// sensor: µs — so fixing it fixes DEPTH exposure too). power_line_freq
// 50/60 enables the RGB module's anti-flicker for that mains frequency
// (0 = leave the device default; D405 has no such option — skipped).
struct ColorExposureConfig {
    // Set when a color_exposure block exists in the config. With it set,
    // auto_exposure is applied EXPLICITLY either way — auto: true re-enables
    // AE left off by a previous run (the device keeps its last UVC controls
    // until power-cycled). Without it the device is left untouched.
    bool configured      = false;
    bool auto_exposure   = true;
    int  exposure        = 0;    // used when auto_exposure=false and > 0
    int  gain            = -1;   // used when auto_exposure=false and >= 0
    int  power_line_freq = 0;    // 0 = leave, else 50 or 60
    // Tone/exposure helpers, applied regardless of auto_exposure:
    int  gamma           = 0;    // 0 = leave; D455 RGB 100..500 (300 default) —
                                 // >300 lifts shadows, barely moves highlights
    int  backlight_comp  = -1;   // -1 = leave; 0/1 — helps bright backgrounds
};

struct CameraCaptureConfig {
    // RealSense device serial to open. Empty = first available device. Required
    // to disambiguate when several cameras are attached (multi-camera setups).
    std::string serial;

    int  depth_width  = 640;
    int  depth_height = 480;
    int  depth_fps    = 30;

    bool color_enabled = true;
    int  color_width   = 640;
    int  color_height  = 480;
    int  color_fps     = 30;

    bool align_to_color = true;  // reproject depth into the color frame

    SpatialFilterConfig spatial_filter;
    ColorExposureConfig color_exposure;

    std::string depth_frame_id = "camera_depth";
    std::string color_frame_id = "camera_color";
};

// Camera device reader (Thread) — the RealSense counterpart of
// UwbSerialReader. Owns a worker that pulls framesets from a librealsense
// pipeline and writes raw ColorFrame / DepthFrame to buffers. No codec,
// no DDS here; encode+publish threads consume these buffers.
class CameraCapture {
public:
    CameraCapture();
    ~CameraCapture();

    bool start(const CameraCaptureConfig& config);
    void stop();

    DataBuffer<DepthFrame> depth_buf;
    DataBuffer<ColorFrame> color_buf;

private:
    void run();

    CameraCaptureConfig      config_;
    std::unique_ptr<rs2::pipeline> pipe_;
    float                    depth_scale_ = 0.001f;
    std::atomic<bool>        running_{false};
    std::atomic<uint64_t>    sequence_{0};
    std::thread              worker_;
};

} // namespace kist
