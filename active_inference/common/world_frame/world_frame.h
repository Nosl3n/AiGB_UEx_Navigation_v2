#pragma once
/*
 * common/world_frame/world_frame.h — which node is the robot's frame in this graph: a `room` indoors, a `field`
 * in the open field (openfield_concept, agricultural worlds).
 *
 * ★2026-11-04: the open-field frame IS the `field` (type and name `field`). The robot exists by itself and then
 * locates itself in a field: the field is both the frame every plant/row/plot hangs from and the concept of the
 * place (its plots are its group_member children, its GPS georeference places it in a larger map). The
 * intermediate `world` node (2026-11-03) is gone. (Before that, too, the frame was called `field`.)
 *
 * WHY THIS EXISTS. Every consumer used to find the frame with get_nodes_by_type("room") and, worse, by the
 * LITERAL name "room" in transform queries. That hard-wired the whole fleet to one localizer. The open-field
 * localizer publishes a node of type `field` with the same contract — delimiting_polygon_x/y, room_height, the
 * robot->field RT edge — so a consumer only needs to ask which of the two is present, and use its NAME as frame.
 *
 * RESOLUTION ORDER: `field` first, then `room`. A graph never carries both in normal operation.
 *
 * Header-only, no state. The lookups walk the node map, so CACHE the result per cycle in hot loops.
 */
#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <dsr/api/dsr_api.h>

namespace rc::world
{

inline constexpr std::array<std::string_view, 2> kFrameTypes = {"field", "room"};

// True for a node type that IS a world frame (so it is never an obstacle, never drawn as furniture, ...).
inline bool is_frame_type(std::string_view type)
{
    return type == "field" or type == "room";
}

// The world-frame node, if any.
inline std::optional<DSR::Node> frame_node(DSR::DSRGraph &G)
{
    for (const auto t : kFrameTypes)
        if (auto v = G.get_nodes_by_type(std::string(t)); not v.empty())
            return v.front();
    return std::nullopt;
}

// The same, as the vector get_nodes_by_type() returns (drop-in for get_nodes_by_type("room")).
inline std::vector<DSR::Node> frame_nodes(DSR::DSRGraph &G)
{
    for (const auto t : kFrameTypes)
        if (auto v = G.get_nodes_by_type(std::string(t)); not v.empty())
            return v;
    return {};
}

// Its name, for transform queries. `fallback` when the graph has none yet ("room" keeps the indoor
// behaviour of every caller that used to hard-code it).
inline std::string frame_name(DSR::DSRGraph &G, std::string fallback = "room")
{
    if (const auto n = frame_node(G); n.has_value())
        return n->name();
    return fallback;
}

// Per-object cache for hot loops: re-resolves only when the cached node has disappeared (or was never
// found), so the common case is one id lookup instead of a walk over every node.
class FrameCache
{
public:
    // const: resolving the name is a cache refresh, not a state change of the owner (the members are mutable).
    const std::string &name(DSR::DSRGraph &G) const
    {
        if (id_ == 0 or not G.get_node(id_).has_value())
        {
            if (const auto n = frame_node(G); n.has_value()) { id_ = n->id(); name_ = n->name(); }
            else                                              { id_ = 0;       name_ = "room"; }
        }
        return name_;
    }
    std::uint64_t id(DSR::DSRGraph &G) const { name(G); return id_; }

private:
    mutable std::uint64_t id_ = 0;
    mutable std::string name_ = "room";
};

}  // namespace rc::world
