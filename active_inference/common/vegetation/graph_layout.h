#pragma once
// Where the orchard nodes sit in the DSR graph VIEWER (pos_x / pos_y attributes; cosmetic, nothing reads them
// as geometry). Layers below the world frame (field) at (wx, wy):
//
//   world (the frame)
//     field layer    : field_<id> metaconcept (the set of plots), above its plots
//       plot layer   : plot_<id> metaconcepts, each above the middle of its rows
//         row layer  : row_<id> metaconcepts, ordered across the rows (west -> east for N-S rows)
//           plants   : each row's plant_<id> individuals in a column under their row, ordered along the row
//   unassigned zone  : plants in no row, below everything
// (A plant's parts — trunk, canopy, ... — are attributes of its node, not nodes: nothing more to lay out.)
//
// plant_concept places a new plant in the unassigned zone; row_metaconcept places the rows and moves a plant under
// its row when it joins one (back to the unassigned zone when it leaves every row); plot_metaconcept places each
// plot over its rows; the field (the frame node itself, openfield_concept) sits above its plots.

#include <utility>

namespace rc::veg::layout
{
inline constexpr float kRowX0 = 200.f;      // first row column, right of the world node
inline constexpr float kRowDX = 110.f;      // between row columns
inline constexpr float kFieldY = 80.f;      // field layer, below the world node
inline constexpr float kPlotY = 160.f;      // plot layer
inline constexpr float kRowY = 240.f;       // row layer
inline constexpr float kTrunkY0 = 320.f;    // first plant under its row
inline constexpr float kTrunkDY = 28.f;     // between plants of a row
inline constexpr float kUnassignedY = 700.f;   // below the deepest column (~14 plants per row)

// k = the row's rank across the rows (0 = westmost).
inline std::pair<float, float> row(float wx, float wy, int k) { return {wx + kRowX0 + kRowDX * k, wy + kRowY}; }

// j = the plant's rank along its row (0 = southmost).
inline std::pair<float, float> row_member(float wx, float wy, int k, int j)
{
    return {wx + kRowX0 + kRowDX * k, wy + kTrunkY0 + kTrunkDY * j};
}

// A plant in no row, by its obj_id.
inline std::pair<float, float> unassigned_plant(float wx, float wy, int id)
{
    return {wx + kRowX0 + 25.f * static_cast<float>(id % 16), wy + kUnassignedY + 25.f * static_cast<float>(id / 16)};
}

}  // namespace rc::veg::layout
