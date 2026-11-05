#pragma once
// CameraPublisher — puts the drone's RGB camera on the zero-copy DDS media plane (rc::media, FastDDS),
// the same transport and frame type the ZED uses (zed_camera/src/dds_publisher.*), so every consumer that
// already reads rc/zed/rgb can read rc/drone/rgb unchanged.
//
// It does NOT touch DSR. Consumers discover the plane through the media descriptor: drone_bridge serves it
// over MediaPlaneDDS::getMediaDescriptor(), and robot_concept relays it onto the `camera` graph node.
// FastDDS headers stay behind a PIMPL. Not thread-safe: publish from one thread.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

class CameraPublisher
{
public:
    struct Config
    {
        std::uint32_t domain_id          = 7;              // CORTEX media domain (never the DSR domain 0)
        std::string   rgb_topic          = "rc/drone/rgb";
        int           history_depth      = 2;
        bool          shared_memory_only = true;
        bool          data_sharing       = false;          // churn-safe: the drone comes and goes
        // Camera intrinsics, appended to the descriptor so the graph node can carry them.
        int           width  = 0;
        int           height = 0;
        double        hfov_rad = 0.0;
    };

    CameraPublisher();
    ~CameraPublisher();
    CameraPublisher(const CameraPublisher&) = delete;
    CameraPublisher& operator=(const CameraPublisher&) = delete;

    bool init(const Config& cfg);
    [[nodiscard]] bool ready() const { return ready_; }
    [[nodiscard]] std::string descriptor_json() const;

    // `rgb` is tightly packed RGB8, width*height*3 bytes.
    bool publish_rgb(std::uint64_t stamp_ms, int width, int height, const std::uint8_t* rgb);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    Config cfg_;
    bool ready_ = false;
};
