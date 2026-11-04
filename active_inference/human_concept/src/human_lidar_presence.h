/*
 * human_lidar_presence.h — the LiDAR as a "second look" at every person the camera has found.
 *
 * The ZED sees ~110° ahead and loses people to occlusion, to the edge of the frame and to YOLO-pose
 * flicker; the helios sees 360° every sweep. Indoors the Shadow covered that gap with the ricoh 360
 * peripheral channel (a bearing that CONFIRMS a known instance, never creates one). The Husky has no 360
 * camera, so the helios plays that role here, and it can do more than a bearing because it has range:
 *
 *  1. EXISTENCE. Per person a log-odds P(exists) (common/existence_belief, the fleet's one removal policy).
 *     A fresh skeleton is evidence FOR. The LiDAR carves a torso box at the believed position: returns inside
 *     it are FOR, beams that cross it are AGAINST, and no beam reaching it (occluded, out of range) is
 *     NO evidence — HOLD. The person is removed when P(exists) is low, not after N missed camera frames, so a
 *     worker the camera stops seeing stays while the LiDAR still finds them, and goes as soon as the beams
 *     pass through where they stood.
 *
 *  2. FOLLOW. While the camera is not seeing a person, the torso-band returns near the believed position
 *     re-centre it, so the belief walks with the worker and the next skeleton re-associates to the SAME track
 *     instead of birthing a new one (the identity churn: 35 nodes for 2 workers on 2026-10-01).
 *
 * The torso band [BandZMin, BandZMax] is above the young palms' crown top (0.95 m), so in this field a palm
 * can neither confirm a person nor occlude one. A crouching worker drops below it: then the LiDAR has nothing
 * to say (HOLD) and only the camera and the no-evidence timeout act — known limit, worth a lower band once
 * vegetation is classified.
 */
#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "human_config.h"
#include "human_instance.h"
#include "../../common/vegetation/plant_discs.h"

namespace rc {

class HumanLidarPresence
{
public:
    explicit HumanLidarPresence(const HumanConfig& cfg) : cfg_(cfg) {}

    // A fresh skeleton was fitted for this person: evidence FOR, and a position fix.
    void on_camera_seen(HumanInstance& inst, std::int64_t now_ms) const;

    // One new sweep (world frame) against every person: follow, then carve. Returns how many persons were
    // re-centred by the LiDAR this sweep.
    int step(std::unordered_map<std::uint64_t, HumanInstance>& instances,
             const std::vector<Eigen::Vector3f>& sweep, const Eigen::Vector3f& origin, std::int64_t now_ms,
             float robot_yaw_rate = 0.0f);

    // Believed torso centre (world xy) — the published pelvis, or nullopt before the first fit.
    static std::optional<Eigen::Vector2f> centre_of(const HumanInstance& inst);

    // The plants (vegetation_concept). A return inside a plant's disc is the PLANT's, never a person's: it
    // neither confirms nor follows anyone. In an orchard the canopy reaches into the torso band, and its leaves
    // next to a worker's believed position used to keep a phantom person alive (and drag it into the row).
    void set_plants(rc::veg::PlantIndex plants) { plants_ = std::move(plants); }

private:
    const HumanConfig& cfg_;
    rc::veg::PlantIndex plants_;
    std::int64_t prev_sweep_ms_ = 0;
};

}  // namespace rc
