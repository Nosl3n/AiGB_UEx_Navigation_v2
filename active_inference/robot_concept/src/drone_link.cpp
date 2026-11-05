#include "drone_link.h"

#include <charconv>
#include <cmath>
#include <optional>
#include <vector>

#include <QDebug>

#include "graph_safe.h"

namespace
{
// A numeric value of a flat JSON key ("width":1280). std::from_chars: locale-independent (CLAUDE.md, es_ES).
std::optional<double> json_number(const std::string& json, const std::string& key)
{
    const std::string tag = "\"" + key + "\":";
    const auto pos = json.find(tag);
    if (pos == std::string::npos)
        return std::nullopt;
    const char* b = json.data() + pos + tag.size();
    const char* e = json.data() + json.size();
    while (b < e and *b == ' ') ++b;
    double v = 0.0;
    const auto [ptr, ec] = std::from_chars(b, e, v);
    if (ec != std::errc{} or ptr == b)
        return std::nullopt;
    return v;
}

// cortex convention: R = Rx(a) * Ry(b) * Rz(c).
std::vector<float> euler_xyz(const RoboCompFullPoseEstimation::FullPoseMatrix& m)
{
    const double b = std::asin(std::clamp(static_cast<double>(m.m02), -1.0, 1.0));
    return {static_cast<float>(std::atan2(-m.m12, m.m22)), static_cast<float>(b),
            static_cast<float>(std::atan2(-m.m01, m.m00))};
}
}  // namespace

DroneLink::DroneLink(std::shared_ptr<DSR::DSRGraph> G,
                     RoboCompMediaPlaneDDS::MediaPlaneDDSPrxPtr media,
                     RoboCompFullPoseEstimation::FullPoseEstimationPrxPtr pose,
                     Config cfg)
    : G_(std::move(G)), media_(std::move(media)), pose_(std::move(pose)), cfg_(std::move(cfg))
{
    if (media_) media_ = media_->ice_invocationTimeout(cfg_.ice_timeout_ms);
    if (pose_)  pose_  = pose_->ice_invocationTimeout(cfg_.ice_timeout_ms);
}

DroneLink::~DroneLink() { stop(); }

void DroneLink::start()
{
    if (thread_.joinable() or not media_ or not pose_)
        return;
    // A crashed / SIGKILLed previous run may have left our nodes in the persistent graph.
    remove_nodes();
    thread_ = std::thread(&DroneLink::loop, this);
}

void DroneLink::stop()
{
    stop_ = true;
    if (thread_.joinable())
        thread_.join();
    if (present_.exchange(false))
        remove_nodes();
}

void DroneLink::loop()
{
    using clock = std::chrono::steady_clock;
    auto last_ok   = clock::now();
    auto last_desc = clock::now();
    while (not stop_)
    {
        if (not present_)
        {
            std::string desc;
            try { desc = media_->getMediaDescriptor(); } catch (const Ice::Exception&) {}
            if (not desc.empty() and create_nodes(desc))
            {
                present_ = true;
                last_ok = last_desc = clock::now();
                qInfo().noquote() << "[drone] drone detected: nodes" << QString::fromStdString(cfg_.drone_node)
                                  << "+" << QString::fromStdString(cfg_.camera_node) << "added under"
                                  << QString::fromStdString(cfg_.parent_node);
            }
            for (int i = 0; i < cfg_.probe_ms / 50 and not stop_; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        bool ok = false;
        try
        {
            const auto drone  = pose_->getFullPoseEuler();
            const auto camera = pose_->getFullPoseMatrix();
            ok = write_poses(drone, camera);
        }
        catch (const Ice::Exception&) {}   // includes HardwareFailedException (no pose yet / simulation over)
        const auto now = clock::now();
        if (ok)
            last_ok = now;
        if (now - last_desc > std::chrono::milliseconds(cfg_.descriptor_ms))
        {
            last_desc = now;
            try
            {
                if (const auto desc = media_->getMediaDescriptor(); not desc.empty())
                {
                    update_descriptor(desc);
                    last_ok = now;
                }
            }
            catch (const Ice::Exception&) {}
        }
        if (now - last_ok > std::chrono::milliseconds(cfg_.lost_ms))
        {
            qInfo() << "[drone] drone lost (no answer for" << cfg_.lost_ms << "ms): removing its nodes";
            present_ = false;
            remove_nodes();
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.poll_ms));
    }
}

bool DroneLink::create_nodes(const std::string& descriptor)
{
    auto parent = G_->get_node(cfg_.parent_node);
    if (not parent.has_value())
    {
        qWarning() << "[drone] no" << QString::fromStdString(cfg_.parent_node) << "node: cannot attach the drone";
        return false;
    }
    auto rt = G_->get_rt_api();
    const auto now_ms = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    auto dron = G_->get_node(cfg_.drone_node);
    if (not dron.has_value())
    {
        DSR::Node n = DSR::Node::create<vehicle_node_type>(cfg_.drone_node);
        G_->add_or_modify_attrib_local<color_att>(n, std::string("SteelBlue"));
        const auto id = G_->insert_node(n);
        if (not id.has_value())
        {
            qWarning() << "[drone] failed to insert the" << QString::fromStdString(cfg_.drone_node) << "node";
            return false;
        }
        dron = G_->get_node(id.value());
        if (not dron.has_value()) return false;
        // Placeholder until the first pose arrives (a few ms later): on the parent's origin.
        rt->insert_or_assign_edge_RT(parent.value(), id.value(), std::vector<float>{0.f, 0.f, 0.f},
                                     std::vector<float>{0.f, 0.f, 0.f}, now_ms);
    }

    if (not G_->get_node(cfg_.camera_node).has_value())
    {
        DSR::Node c = DSR::Node::create<camera_node_type>(cfg_.camera_node);
        G_->add_or_modify_attrib_local<color_att>(c, std::string("SkyBlue"));
        const auto id = G_->insert_node(c);
        if (not id.has_value())
        {
            qWarning() << "[drone] failed to insert the" << QString::fromStdString(cfg_.camera_node) << "node";
            return false;
        }
        auto d = G_->get_node(cfg_.drone_node);
        if (not d.has_value()) return false;
        rt->insert_or_assign_edge_RT(d.value(), id.value(), std::vector<float>{0.f, 0.f, 0.f},
                                     std::vector<float>{0.f, 0.f, 0.f}, now_ms);
    }
    descriptor_.clear();
    update_descriptor(descriptor);
    return true;
}

void DroneLink::update_descriptor(const std::string& descriptor)
{
    if (descriptor == descriptor_)
        return;   // unchanged: no write, no graph signal
    auto cam = G_->get_node(cfg_.camera_node);
    if (not cam.has_value())
        return;
    G_->add_or_modify_attrib_local<media_descriptor_att>(cam.value(), descriptor);
    // Intrinsics advertised by the driver alongside the transport (pinhole, square pixels).
    const auto w = json_number(descriptor, "width");
    const auto h = json_number(descriptor, "height");
    const auto fov = json_number(descriptor, "hfov_rad");
    if (w and h and fov and *fov > 0.0)
    {
        const int focal = static_cast<int>(std::lround(*w / 2.0 / std::tan(*fov / 2.0)));
        G_->add_or_modify_attrib_local<cam_rgb_width_att>(cam.value(), static_cast<int>(*w));
        G_->add_or_modify_attrib_local<cam_rgb_height_att>(cam.value(), static_cast<int>(*h));
        G_->add_or_modify_attrib_local<cam_fov_att>(cam.value(), static_cast<float>(*fov));
        G_->add_or_modify_attrib_local<cam_rgb_focalx_att>(cam.value(), focal);
        G_->add_or_modify_attrib_local<cam_rgb_focaly_att>(cam.value(), focal);
    }
    rc::safe_update_node(*G_, cam.value());
    descriptor_ = descriptor;
    qInfo().noquote() << "[drone] camera media descriptor:" << QString::fromStdString(descriptor);
}

bool DroneLink::write_poses(const RoboCompFullPoseEstimation::FullPoseEuler& drone,
                            const RoboCompFullPoseEstimation::FullPoseMatrix& camera)
{
    auto parent = G_->get_node(cfg_.parent_node);
    auto dron   = G_->get_node(cfg_.drone_node);
    auto cam    = G_->get_node(cfg_.camera_node);
    if (not parent.has_value() or not dron.has_value() or not cam.has_value())
        return false;
    auto rt = G_->get_rt_api();
    const auto ts_d = static_cast<std::uint64_t>(drone.timestamp);
    const auto ts_c = static_cast<std::uint64_t>(camera.timestamp);
    rt->insert_or_assign_edge_RT(parent.value(), dron->id(), std::vector<float>{drone.x, drone.y, drone.z},
                                 std::vector<float>{drone.rx, drone.ry, drone.rz}, ts_d);
    rt->insert_or_assign_edge_RT(dron.value(), cam->id(), std::vector<float>{camera.m03, camera.m13, camera.m23},
                                 euler_xyz(camera), ts_c);
    return true;
}

void DroneLink::remove_nodes()
{
    for (const auto& name : {cfg_.camera_node, cfg_.drone_node})
        if (auto n = G_->get_node(name); n.has_value())
        {
            // Only OUR kind of node: never delete somebody else's "camera".
            const bool ours = (name == cfg_.camera_node and n->type() == "camera")
                              or (name == cfg_.drone_node and n->type() == "vehicle");
            if (ours)
                G_->delete_node(n.value());
        }
    descriptor_.clear();
}
