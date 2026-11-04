/*
 *    Copyright (C) 2026 by RoboLab at the University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

// openfield_concept — robot localization in an OPEN FIELD; the room_concept replacement for
// agricultural worlds. GPS (bridge, Ice) + IMU (media plane) + wheel odometry (robot node) are fused
// by an SE(2) EKF (openfield_ekf.h), and the result is published with room_concept's exact DSR
// contract, with a "field" node (not "room") carrying the field polygon, the robot->field RT edge, the field's plots
// (group_member) and the inspection state. The robot exists by itself and then locates itself in the field.

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include "openfield_ekf.h"
#include "openfield_variants.h"
#include "openfield_scene_graph.h"
#include "openfield_viewer.h"
#include "openfield_world3d.h"
#include "openfield_exploration.h"

#include "../../common/agent_presence_coordinator/agent_presence_coordinator.h"
#include "../../common/concept_presence/concept_presence.h"
#include "../../common/media_transport/media_transport.h"

#include <atomic>
#include <deque>
#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <fps/fps.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
	~SpecificWorker();

public slots:
	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();

	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	void modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){};
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};

private:
	bool startup_check_flag = false;

	// ── Config ───────────────────────────────────────────────────────────────────────────────────
	struct Config
	{
		rc::openfield::EkfParams ekf;
		std::vector<double> poly_x, poly_y;        // field boundary, local metres (x east, y north)
		std::vector<double> poly_lat, poly_lon;    // ...or as GPS coordinates (degrees)
		// Field.Source = "webots": the boundary is derived from the loaded world (bridge getWorldExtents):
		// bbox of the ObjectTypes nodes + margins, grown to hold the start pose, clipped to the TerrainTypes.
		std::string field_source = "config";       // "config" | "webots"
		std::vector<std::string> object_types{"AppleTree"};
		std::vector<std::string> terrain_types{"FieldTerrain", "RectangleArena"};
		double margin_x = 3.0, margin_y = 6.0;     // m around the objects (headlands to turn in)
		double start_margin = 2.5;                 // m kept around the robot's start position
		float world_height = 5.0f;                  // obstacle-cloud ceiling published on the room node
		int yaw_update_period_ms = 100;            // IMU absolute-yaw correction rate (sim ms)
		bool use_imu_yaw = true;
		int gps_poll_period_ms = 50;
		int imu_stall_timeout_ms = 2000;           // primary-input gate (rc::stream)
		std::string imu_node = "imu";
		std::string csv_path = "tmp/openfield_pose.csv";
		bool viewer = true;                        // Viewer.Enabled: the localization window
		bool world3d = true;                       // Viewer.World3D: the 3D semantic map window
		rc::openfield::OpenFieldExploration::Config exploration;   // [Exploration]: the inspection mode
		// GPS fusion model of the PUBLISHED filter ([Ekf] GpsSigmaSource / GpsLikelihood / StudentNu /
		// LagCompensation). Defaults = the filter as it always was. See openfield_variants.h.
		rc::openfield::GpsModel gps_model;
		std::string gps_status_file;               // [Gps] StatusFile: the receiver's report (bridge, simulation)
		bool shadow_filters = false;               // [Experiment] ShadowFilters: E1's A/B/C/D side by side
		std::string exp_csv_path;                  // [Diag] ExperimentCsv: 10 Hz, every filter + GT + receiver
	} cfg_;
	void load_config();

	// ── Presence (shared protocol, common/concept_presence) ──────────────────────────────────────
	AgentPresenceCoordinator presence_coordinator_;
	rc::presence::ConceptProtocol presence_protocol_;
	std::atomic<bool> shutting_down_{false};
	bool owned_nodes_cleaned_ = false;
	void waiting_enter()   { presence_coordinator_.waiting_enter(); }
	void waiting_loop()    { presence_coordinator_.waiting_loop(); }
	void operating_enter() { presence_coordinator_.operating_enter(); }
	void operating_loop()  { presence_coordinator_.operating_loop(); }
	void degraded_enter()  { presence_coordinator_.degraded_enter(); }
	void degraded_loop()   { presence_coordinator_.degraded_loop(); }
	void request_shutdown();
	void terminal_shutdown();

	// ── Inputs ───────────────────────────────────────────────────────────────────────────────────
	// IMU (media plane): the primary input, and the clock the filter is propagated on.
	std::unique_ptr<rc::media::ImuSubscriber> imu_sub_;
	std::int64_t last_imu_try_ms_ = 0;
	std::int64_t last_imu_recv_ms_ = -1;       // wall ms of the last frame received (for the gate)
	std::int64_t last_imu_clock_ms_ = 0;       // sim ms (or wall ms on hardware) of the last propagated frame
	std::int64_t last_imu_stamp_ms_ = 0;       // wall epoch ms of the last frame (the RT edge timestamp)
	std::int64_t last_yaw_update_clock_ms_ = 0;
	double last_gyro_z_ = 0.0;
	std::optional<double> last_imu_yaw_;
	void pump_imu();
	[[nodiscard]] std::int64_t imu_age_ms() const;

	// Wheel odometry (robot node attributes written by robot_concept).
	std::uint64_t last_odom_ts_ = 0;
	float odom_adv_ = 0.f, odom_side_ = 0.f, odom_rot_ = 0.f;
	Eigen::Vector3f odom_var_{0.f, 0.f, 0.f};
	void read_odometry();

	// GPS (bridge, Ice).
	std::int64_t last_gps_poll_ms_ = 0;
	std::optional<Eigen::Vector2d> last_gps_xy_;
	std::uint64_t gps_fixes_ = 0, gps_fused_ = 0;
	std::int64_t last_gps_new_ms_ = 0;         // wall ms of the last NEW fix
	double last_gps_innovation_m_ = 0.0;
	void poll_gps();
	// Receiver status file ([Gps] StatusFile): one line per fix, replaced atomically by the bridge.
	std::uint64_t last_status_seq_ = 0;
	rc::openfield::GpsFix last_fix_;
	bool read_status_fix(rc::openfield::GpsFix &fix);
	rc::openfield::AntennaHistory main_antenna_;
	rc::openfield::GpsDiag main_diag_;
	// E1 shadow filters and their log.
	std::vector<rc::openfield::Shadow> shadows_;
	std::ofstream exp_csv_;
	std::int64_t last_exp_row_ms_ = 0;
	void write_experiment_row();

	// ── Sensor switches (the window): take the GPS / the IMU's absolute yaw out of the filter ─────
	bool gps_enabled_ = true;
	bool yaw_enabled_ = true;
	std::int64_t gps_off_since_ms_ = 0;        // 0 while the GPS is on
	std::unique_ptr<rc::openfield::OpenFieldViewer> viewer_;
	// The 3D semantic map (openfield_world3d.h): the robot, the plants, rows, plots and field, rebuilt from the graph.
	std::unique_ptr<rc::viewers::GLWorld3DViewer> world3d_;
	std::unique_ptr<DSR::InnerEigenAPI> world3d_eigen_;   // main-thread instance (its ts==0 cache is ours)
	void push_world3d();
	std::int64_t start_ms_ = 0;
	std::uint64_t compute_ticks_ = 0;
	void push_viewer();

	// ── Estimation and publication ───────────────────────────────────────────────────────────────
	rc::openfield::OpenFieldEkf ekf_;
	std::unique_ptr<DSR::RT_API> rt_api_;
	std::unique_ptr<rc::openfield::OpenFieldSceneGraph> scene_graph_;
	std::vector<Eigen::Vector2f> field_polygon_;
	bool resolve_field_polygon();              // needs a GPS fix when the boundary is given in lat/lon
	std::optional<std::vector<Eigen::Vector2f>> field_polygon_from_webots();   // Field.Source = "webots"
	void publish_pose();

	// ── Inspection mode (openfield_exploration.h): afford_openfield + the world node's model_stable ──────────
	std::unique_ptr<rc::openfield::OpenFieldExploration> exploration_;

	// ── Diagnostics ──────────────────────────────────────────────────────────────────────────────
	std::ofstream csv_;
	// Ground truth graded at its own stamp (see pose_history_). Simulation only.
	struct GtError { Eigen::Vector3d gt; double err_xy = 0, err_theta = 0; bool stamped = false; };
	[[nodiscard]] std::optional<GtError> gt_error() const;
	// Published poses by timestamp, so the ground truth is graded at ITS OWN instant. It arrives at the
	// base's 10 Hz; comparing it with the newest estimate instead charged up to 100 ms of motion to the
	// filter (2.9 deg at 0.5 rad/s), which is the comparison's error, not the localizer's.
	std::deque<std::pair<std::int64_t, Eigen::Vector3d>> pose_history_;
	[[nodiscard]] std::optional<Eigen::Vector3d> pose_at(std::int64_t stamp_ms) const;
	std::int64_t last_status_ms_ = 0;
	std::uint64_t published_ = 0;
	void log_status();
	FPSCounter fps_;

signals:
	void presenceReady();
	void presenceLost();
};

#endif
