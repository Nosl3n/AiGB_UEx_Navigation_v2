#pragma once
/*
 * MixedPixelFilter — drops LiDAR returns that agree with NONE of their neighbours in the scan grid.
 *
 * WHY. At a depth edge (a leaf against the ground behind it, or against the sky) a return can come back with a
 * range BETWEEN the two surfaces: a point floating in the air where nothing is. The simulated helios does it at
 * every leaf edge (Webots renders depth images and resamples them into beams), and real lidars do it too
 * (beam footprint over two surfaces). MEASURED 2026-10-15 in husky_apple.wbt, 20 sweeps scored against the
 * exact world geometry: ~220 returns/sweep floating 0.6-2.5 m up with nothing there; 69% of them on a beam
 * that grazes a leaf before reaching them (2% for genuine ground returns), the rest just IN FRONT of a canopy
 * seen against the sky (1.8-3.5 m from the robot). Static scene => static phantoms => residual_concept
 * accumulated them as obstacle cells in the corridors (43/sweep in the 0.25-1.2 m band).
 *
 * RULE. A real surface is seen by more than one beam: at least one of its 8 grid neighbours (same ring +-1
 * column, adjacent rings' nearest column +-1) lies at a similar range, |dr| < max(abs_tol, rel_tol * r).
 * A mixed pixel sits alone between two surfaces and agrees with neither. Offline on those sweeps
 * (rel 5%, abs 0.10 m): removes 49% of the floating phantoms and 84% of the near ones, 0% of the ground,
 * and 10% of the true tree returns (single leaves seen against the background — the canopy keeps the rest).
 *
 * GRID FROM THE DATA, not from a configured resolution: rings are the distinct beam elevations of the sweep
 * (clustered with a 0.5 deg gap, like the ring table in specificworker.cpp), and the azimuth step is each
 * ring's own median spacing. So it works on the uniform simulated fan and on the real helios's uneven rings.
 * Run it on the full device-frame cloud, BEFORE the mesh self-filter: the self-filter's holes would otherwise
 * read as "no neighbour" and take the points around the robot's silhouette with them.
 */
#include <Lidar3D.h>
#include <string>

struct MixedPixelFilterConfig
{
    float rel_tol = 0.05f;     // neighbour agrees if |dr| < max(abs_tol_mm, rel_tol * r)
    float abs_tol_mm = 100.f;  // points are in mm here (device frame, see processLidarData)
};

// Removes the mixed pixels from `data.points` (order preserved). Returns how many were removed.
std::size_t mixed_pixel_filter(RoboCompLidar3D::TData& data, const MixedPixelFilterConfig& cfg);

// Synthetic check: a wall with a pole in front and one mixed pixel between them. True on PASS.
bool mixed_pixel_filter_self_test(std::string* report = nullptr);
