// openfield_scene_graph.h — the DSR side of openfield_concept: the `field` node (the robot's frame in the open field
// and the concept of the place) and the robot->field RT.
//
// ★ THE CONTRACT IS room_concept's, BYTE FOR BYTE, with one difference: the frame node is of TYPE and NAME `field`
// (cortex dsr_node_type.h), not `room`. Consumers resolve it through common/world_frame (rc::world::frame_node /
// FrameCache), which accepts either, so controller and residual_concept drive on a field exactly as they do in a
// room: same delimiting_polygon_x/y, same room_height, same RT edge layout robot->field. The edge writer is a port of
// RoomSceneGraph::write_robot_room_rt
// (room_concept/src/room_scene_graph.cpp); read the comments there before changing any index or sign —
// each one records a bug that already cost a measurement to find:
//   * the edge is parent=ROBOT, child=FIELD, so it stores T_robot<-field (the INVERSE of the pose), and the
//     covariance goes through the inversion Jacobian;
//   * rt_covariance is a 6x6 row-major SE(3) block [x,y,z,rx,ry,rz]: SE(2) goes to indices 0, 1, 5;
//   * the ring twist is the CHILD's (the field's) motion in the child's axes = -Ad(v_body, w), AXIS order;
//   * the legacy rt_translation_velocity pair is the robot's body twist in ARRAY order [adv, side, 0],
//     and it is written BEFORE the timestamped RT block ("ORDER IS LOAD-BEARING").
#pragma once

#include <genericworker.h>   // DSR API + generated node/attr type tags

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <set>
#include <vector>

namespace rc::openfield
{

struct PoseSample
{
    double x = 0, y = 0, theta = 0;                       // field<-robot, see openfield_ekf.h
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();    // over [x, y, theta]
    float adv = 0.f, side = 0.f, rot = 0.f;               // body twist (m/s, m/s, rad/s)
    Eigen::Vector3f vel_var{0.f, 0.f, 0.f};               // variances of [adv, side, rot]
    std::uint64_t timestamp_ms = 0;                       // WALL epoch ms of validity
};

class OpenFieldSceneGraph
{
public:
    OpenFieldSceneGraph(std::shared_ptr<DSR::DSRGraph> G, DSR::RT_API *rt_api);

    // Create the room node, or ADOPT an existing one (persisted graph / previous run) and make sure it
    // carries the polygon. Idempotent. Returns true once the room exists.
    bool ensure_world(const std::vector<Eigen::Vector2f> &polygon, float world_height);
    [[nodiscard]] bool has_world() const { return world_id_ != 0; }
    [[nodiscard]] std::uint64_t world_id() const { return world_id_; }
    [[nodiscard]] std::uint64_t robot_id();

    // The field's plots as its members (field —group_member→ plot_<id>). Call at a low rate.
    void group_plots();

    // Write the robot->field RT (pose, covariance, twist). Refuses non-finite input.
    bool publish(const PoseSample &s);

private:
    std::shared_ptr<DSR::DSRGraph> G_;
    DSR::RT_API *rt_api_ = nullptr;
    std::uint64_t robot_id_ = 0;
    std::uint64_t world_id_ = 0;              // the field node (the frame)
    std::set<std::uint64_t> members_;         // plot ids the field groups
};

} // namespace rc::openfield
