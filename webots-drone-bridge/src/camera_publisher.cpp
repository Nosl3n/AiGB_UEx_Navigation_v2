#include "camera_publisher.h"

#include <cstring>
#include <format>
#include <print>

#include "media_transport.h"

namespace
{
// The ImageFrame instance key. STREAM_ZED_RGB(0)/DEPTH(1)/IMU(2)/LIDAR(3) are taken; the key only has to be
// unique within a topic, and this is the only stream on rc/drone/rgb.
constexpr std::uint32_t STREAM_DRONE_RGB = 20;
}

struct CameraPublisher::Impl
{
    rc::media::MediaPublisher rgb;
    std::uint64_t frame_id = 0;
};

CameraPublisher::CameraPublisher() : pimpl_(std::make_unique<Impl>()) {}
CameraPublisher::~CameraPublisher() = default;

bool CameraPublisher::init(const Config& cfg)
{
    cfg_ = cfg;
    rc::media::PublisherConfig pc;
    pc.domain_id          = cfg.domain_id;
    pc.topic_name         = cfg.rgb_topic;
    pc.history_depth      = cfg.history_depth;
    pc.shared_memory_only = cfg.shared_memory_only;
    pc.data_sharing       = cfg.data_sharing;
    ready_ = pimpl_->rgb.init(pc);
    if (ready_)
        std::print("[drone_bridge] DDS media plane ready: domain={} rgb='{}' data_sharing={}\n",
                   cfg.domain_id, cfg.rgb_topic, pimpl_->rgb.data_sharing_active());
    else
        std::print(stderr, "[drone_bridge] DDS media plane init FAILED (topic '{}')\n", cfg.rgb_topic);
    return ready_;
}

std::string CameraPublisher::descriptor_json() const
{
    if (not ready_)
        return {};
    rc::media::MediaDescriptor d;
    d.version            = 1;
    d.domain_id          = cfg_.domain_id;
    d.type_name          = "ImageFrame";
    d.type_tag           = rc::media::IMAGE_FRAME_TYPE_TAG;
    d.history_depth      = cfg_.history_depth;
    d.shared_memory_only = cfg_.shared_memory_only;
    d.data_sharing       = cfg_.data_sharing;
    d.ready              = ready_;
    d.streams["rgb"]     = cfg_.rgb_topic;
    std::string json = d.to_json();
    // Intrinsics ride along as extra flat keys (MediaDescriptor::from_json ignores keys it does not know).
    // Extrinsics do NOT: where the camera is lives in the RT tree (drone -> camera edge).
    if (cfg_.width > 0 and cfg_.height > 0 and json.size() >= 2 and json.back() == '}')
    {
        json.pop_back();
        json += std::format(R"(,"width":{},"height":{},"hfov_rad":{:.6f}}})", cfg_.width, cfg_.height, cfg_.hfov_rad);
    }
    return json;
}

bool CameraPublisher::publish_rgb(std::uint64_t stamp_ms, int width, int height, const std::uint8_t* rgb)
{
    const std::size_t nbytes = static_cast<std::size_t>(width) * height * 3;
    if (not ready_ or rgb == nullptr or nbytes == 0 or nbytes > rc::media::MAX_IMAGE_BYTES)
        return false;
    rc::media::ImageFrame* s = pimpl_->rgb.loan();
    if (s == nullptr)
        return false;   // pool exhausted -> drop this frame
    s->stream_id(STREAM_DRONE_RGB);
    s->frame_id(pimpl_->frame_id++);
    s->stamp_ms(stamp_ms);
    s->width(static_cast<std::uint32_t>(width));
    s->height(static_cast<std::uint32_t>(height));
    s->step(static_cast<std::uint32_t>(width * 3));
    s->format(rc::media::FORMAT_RGB8);
    s->size(static_cast<std::uint32_t>(nbytes));
    std::memcpy(s->data().data(), rgb, nbytes);
    return pimpl_->rgb.publish(s);
}
