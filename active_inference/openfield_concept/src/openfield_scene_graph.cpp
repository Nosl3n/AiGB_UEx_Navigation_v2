#include "openfield_scene_graph.h"

#include "../../common/graph_provenance/creation_stamp.h"   // rc::provenance::stamp_creation

#include <QDebug>
#include <cmath>

namespace rc::openfield
{

OpenFieldSceneGraph::OpenFieldSceneGraph(std::shared_ptr<DSR::DSRGraph> G, DSR::RT_API *rt_api)
    : G_(std::move(G)), rt_api_(rt_api)
{}

std::uint64_t OpenFieldSceneGraph::robot_id()
{
    if (robot_id_ != 0 and G_->get_node(robot_id_).has_value())
        return robot_id_;
    const auto robots = G_->get_nodes_by_type("robot");
    robot_id_ = robots.empty() ? 0 : robots.front().id();
    return robot_id_;
}

bool OpenFieldSceneGraph::ensure_world(const std::vector<Eigen::Vector2f> &polygon, float world_height)
{
    if (world_id_ != 0 and G_->get_node(world_id_).has_value())
        return true;
    world_id_ = 0;

    std::vector<float> px, py;
    for (const auto &v : polygon) { px.push_back(v.x()); py.push_back(v.y()); }

    // A `world` node is the frame of the 2026-11-03 contract, now gone: if one survives from an older run, it is
    // stale — every child would hang from the wrong frame. Remove it (we owned it).
    for (const auto &stale : G_->get_nodes_by_type("world"))
    {
        qInfo() << "[openfield] removing a stale 'world' node" << stale.id() << "(the frame is now the field)";
        G_->delete_node(stale.id());
    }

    // ADOPT: a field node may survive from a previous run (persisted graph) or a crashed one. Make sure
    // it carries OUR polygon — every consumer's containment prior reads it.
    if (const auto rooms = G_->get_nodes_by_type("field"); not rooms.empty())
    {
        auto rn = rooms.front();
        world_id_ = rn.id();
        G_->add_or_modify_attrib_local<delimiting_polygon_x_att>(rn, px);
        G_->add_or_modify_attrib_local<delimiting_polygon_y_att>(rn, py);
        G_->add_or_modify_attrib_local<room_height_att>(rn, world_height);
        G_->update_node(rn);
        qInfo() << "[openfield] adopted existing field node" << world_id_ << "and set its polygon ("
                << static_cast<int>(px.size()) << "vertices)";
        return true;
    }

    // Type AND name `field`: the robot's frame in the open field AND the concept of the place (its plots are its
    // group_member children; active_inference/conceptos.md). Consumers find it through common/world_frame
    // (rc::world::frame_node), which prefers `field` and falls back to `room`.
    DSR::Node world = DSR::Node::create<field_node_type>("field");
    G_->add_or_modify_attrib_local<delimiting_polygon_x_att>(world, px);
    G_->add_or_modify_attrib_local<delimiting_polygon_y_att>(world, py);
    // Outdoors there is no ceiling; this is the obstacle-cloud ceiling consumers crop the LiDAR with.
    G_->add_or_modify_attrib_local<room_height_att>(world, world_height);
    // Width 0 = "stated, never measured" (room_concept's convention for a typed-in height).
    G_->add_or_modify_attrib_local<room_height_sigma_att>(world, 0.f);
    rc::provenance::stamp_creation(*G_, world);
    const auto id = G_->insert_node(world);
    if (not id.has_value())
    {
        qWarning() << "[openfield] failed to insert the field node";
        return false;
    }
    world_id_ = id.value();
    qInfo() << "[openfield] field node created, id" << world_id_ << "," << static_cast<int>(px.size())
            << "vertices, height" << world_height << "m";
    return true;
}

// The field's MEMBERS: every plot (plot_metaconcept) is a part of this field, `field —group_member→ plot_<id>`.
// This was field_metaconcept's job when the field was a separate node; now the field is the frame, and its owner
// groups its plots. Idempotent: adds the missing edges, drops the ones to plots that are gone.
void OpenFieldSceneGraph::group_plots()
{
    if (world_id_ == 0) return;
    std::set<std::uint64_t> want;
    for (const auto &n : G_->get_nodes_by_type("metaconcept"))
        if (n.name().starts_with("plot_")) want.insert(n.id());
    for (const auto to : members_)
        if (not want.contains(to))
            G_->delete_edge(world_id_, to, "group_member");
    for (const auto to : want)
        if (not members_.contains(to) and not G_->get_edge(world_id_, to, "group_member").has_value())
            G_->insert_or_assign_edge(DSR::Edge::create<group_member_edge_type>(world_id_, to));
    if (want != members_)
        qInfo() << "[openfield] the field groups" << static_cast<int>(want.size()) << "plots";
    members_ = std::move(want);
}

bool OpenFieldSceneGraph::publish(const PoseSample &s)
{
    if (not G_ or not rt_api_ or world_id_ == 0) return false;

    // SAFETY BARRIER (room_concept's): a NaN on this edge reaches the controller that drives the wheels.
    if (not (std::isfinite(s.x) and std::isfinite(s.y) and std::isfinite(s.theta) and s.cov.allFinite()))
    {
        static std::uint64_t k = 0;
        if ((k++ % 20) == 0)
            qCritical() << "[SAFETY] refusing to publish a NON-FINITE robot pose; keeping the last good edge";
        return false;
    }

    auto parent = G_->get_node(robot_id());
    if (not parent.has_value()) return false;
    if (not G_->get_node(world_id_).has_value()) { world_id_ = 0; return false; }

    // field<-robot pose -> robot<-field (the stored edge).
    const Eigen::Vector2f t(static_cast<float>(s.x), static_cast<float>(s.y));
    const float th = static_cast<float>(s.theta);
    const Eigen::Matrix2f R = Eigen::Rotation2Df(th).toRotationMatrix();
    const Eigen::Vector2f t_inv = -(R.transpose() * t);
    const float th_inv = -th;

    // Covariance through the inversion Jacobian (room_scene_graph.cpp, same J).
    const float c = std::cos(th), sn = std::sin(th);
    Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
    J(0, 0) = -c;  J(0, 1) = -sn; J(0, 2) =  sn * t.x() - c * t.y();
    J(1, 0) =  sn; J(1, 1) = -c;  J(1, 2) =  c * t.x() + sn * t.y();
    J(2, 2) = -1.f;
    const Eigen::Matrix3f cov_inv = J * s.cov.cast<float>() * J.transpose();

    static constexpr int se3_of_se2[3] = {0, 1, 5};
    std::vector<float> cov_flat(36, 0.f);
    for (int r = 0; r < 3; ++r)
        for (int k = 0; k < 3; ++k)
            cov_flat[se3_of_se2[r] * 6 + se3_of_se2[k]] = cov_inv(r, k);

    // Body twist -> the CHILD's twist (-Ad), AXIS order (x lateral, y forward), with its covariance.
    Eigen::Matrix3f body_vel_cov = Eigen::Matrix3f::Zero();
    body_vel_cov(0, 0) = s.vel_var(1);   // lateral
    body_vel_cov(1, 1) = s.vel_var(0);   // advance (+Y forward)
    body_vel_cov(2, 2) = s.vel_var(2);   // yaw rate
    Eigen::Matrix3f adjoint = Eigen::Matrix3f::Identity();
    adjoint.topLeftCorner<2, 2>() = R;
    adjoint(0, 2) =  t.y();
    adjoint(1, 2) = -t.x();
    const Eigen::Vector2f v_body{s.side, s.adv};
    const Eigen::Vector2f v_child = -(R * v_body + s.rot * Eigen::Vector2f{t.y(), -t.x()});
    const float w_child = -s.rot;
    const Eigen::Matrix3f child_vel_cov = adjoint * body_vel_cov * adjoint.transpose();
    std::vector<float> vel_cov(36, 0.f);
    for (int r = 0; r < 3; ++r)
        for (int k = 0; k < 3; ++k)
            vel_cov[se3_of_se2[r] * 6 + se3_of_se2[k]] = child_vel_cov(r, k);

    // Legacy pair FIRST (array order, robot body twist) — the RT ring write must be the last word.
    if (auto edge = G_->get_edge(parent->id(), world_id_, "RT"); edge.has_value())
    {
        G_->add_or_modify_attrib_local<rt_translation_velocity_att>(edge.value(), std::vector<float>{s.adv, s.side, 0.f});
        G_->add_or_modify_attrib_local<rt_rotation_euler_xyz_velocity_att>(edge.value(), std::vector<float>{0.f, 0.f, s.rot});
        G_->insert_or_assign_edge(edge.value());
    }

    try
    {
        rt_api_->insert_or_assign_edge_RT(parent.value(), world_id_,
                                          DSR::RT_API::RTBlock{
                                              .translation      = {t_inv.x(), t_inv.y(), 0.f},
                                              .rotation_euler   = {0.f, 0.f, th_inv},
                                              .covariance       = cov_flat,
                                              .twist_linear     = std::vector<float>{v_child.x(), v_child.y(), 0.f},
                                              .twist_angular    = std::vector<float>{0.f, 0.f, w_child},
                                              .twist_covariance = vel_cov},
                                          s.timestamp_ms);
    }
    catch (const std::exception &e)
    {
        qWarning() << "[openfield] insert_or_assign_edge_RT failed:" << e.what();
        return false;
    }
    return true;
}

} // namespace rc::openfield
