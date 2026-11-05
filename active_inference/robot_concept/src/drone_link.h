#pragma once
// DroneLink — the inspection drone as part of the robot's body in the graph, ONLY while a drone exists.
//
//   body ──RT──▶ dron (vehicle) ──RT──▶ camera (camera, media_descriptor = rc/drone/rgb)
//
// "A drone exists" is answered by the drone's own driver (webots-drone-bridge / drone_bridge): it serves
//   * MediaPlaneDDS::getMediaDescriptor()  — non-empty while its camera plane is live;
//   * FullPoseEstimation::getFullPoseEuler()  — the drone in the carrier's body frame;
//     FullPoseEstimation::getFullPoseMatrix() — the camera in the drone's frame (gimbal included).
// Nobody answering means no drone: the nodes are never created, and robot_concept behaves exactly as before.
// When the driver stops answering for `lost_ms`, the nodes are removed (camera first, then dron).
//
// Frames follow the robot convention (+Y forward / optical axis, +X right, +Z up), so `camera` reads like `zed`.
// Runs on its own thread: graph reads/writes are thread-safe (active_inference/CLAUDE.md), the Ice calls carry
// a short invocation timeout so a dead peer never stalls the agent, and nothing here touches Qt.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "dsr/api/dsr_api.h"
#include <FullPoseEstimation.h>
#include <MediaPlaneDDS.h>

class DroneLink
{
public:
    struct Config
    {
        std::string parent_node = "body";     // the drone hangs from the robot's body
        std::string drone_node  = "dron";
        std::string camera_node = "camera";
        int probe_ms       = 1000;   // looking for a drone that is not there yet
        int poll_ms        = 100;    // pose refresh while it is there
        int descriptor_ms  = 3000;   // re-read the media descriptor (it may change, e.g. after a restart)
        int lost_ms        = 3000;   // no answer for this long -> the drone is gone (presence heartbeat)
        int ice_timeout_ms = 200;
    };

    DroneLink(std::shared_ptr<DSR::DSRGraph> G,
              RoboCompMediaPlaneDDS::MediaPlaneDDSPrxPtr media,
              RoboCompFullPoseEstimation::FullPoseEstimationPrxPtr pose,
              Config cfg);
    ~DroneLink();
    DroneLink(const DroneLink&) = delete;
    DroneLink& operator=(const DroneLink&) = delete;

    void start();
    void stop();                 // joins the thread and removes the nodes (idempotent)
    [[nodiscard]] bool present() const { return present_.load(); }

private:
    void loop();
    bool create_nodes(const std::string& descriptor);
    void update_descriptor(const std::string& descriptor);
    bool write_poses(const RoboCompFullPoseEstimation::FullPoseEuler& drone,
                     const RoboCompFullPoseEstimation::FullPoseMatrix& camera);
    void remove_nodes();

    std::shared_ptr<DSR::DSRGraph> G_;
    RoboCompMediaPlaneDDS::MediaPlaneDDSPrxPtr media_;
    RoboCompFullPoseEstimation::FullPoseEstimationPrxPtr pose_;
    Config cfg_;
    std::thread thread_;
    std::atomic_bool stop_{false};
    std::atomic_bool present_{false};
    std::string descriptor_;
};
