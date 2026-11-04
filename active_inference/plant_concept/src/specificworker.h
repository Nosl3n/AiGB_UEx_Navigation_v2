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

// plant_concept — the PLANTS of the field as individuals, the way human_concept owns the people.
// A plant is a CONCEPT: one individual, one `plant` node plant_<id>. Its parts are ATTRIBUTES of that node, never
// nodes of their own (the full attribute contract is in active_inference/conceptos.md):
//   * trunk  — trunk_map.h: retina's ADE20K 'tree' masks in the band between the ground and the first branch
//              propose a trunk, the helios confirms it with returns in the same band and places it (~cm). The trunk
//              is the plant's anchor: the world→plant RT stands on it, and it forbids a keep-out disc.
//   * canopy — vegetation_map.h: every vegetation mask point accumulated on a ground-plane evidence grid; a disc of
//              foliage seen from above. What hides things (an occluder for the trunk model), not an obstacle.
// The two sub-models are composed into individuals by plant_composer.h (a trunk inside a canopy's disc is that
// plant's trunk). A plant may be incomplete (a trunk whose canopy is unseen, a canopy whose trunk is hidden); it
// is completed, keeping its id, when the missing part appears. Later parts (leaves, fruit, health) are further
// attributes, added here. row_metaconcept groups the plants into rows: field -> row -> plant.
// (Replaces trunk_concept + vegetation_concept, which published trunk_* and canopy-as-`plant` nodes separately.)

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include "plant_composer.h"
#include "trunk_map.h"
#include "vegetation_map.h"

#include "../../common/agent_presence_coordinator/agent_presence_coordinator.h"
#include "../../common/concept_presence/concept_presence.h"
#include "../../common/lidar_ingestor/concept_lidar_ingestor.h"
#include "../../common/mask_ingestor/mask_ingestor.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>

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
		rc::trunk::TrunkParams trunk;
		rc::veg::VegetationParams canopy;
		float keepout_radius_m = 0.5f;       // the disc around a trunk the robot may not enter
		float person_radius_m = 0.4f;        // a person as an occluder
		std::set<std::string> trunk_labels{"tree", "palm"};                          // ADE20K classes with a trunk
		std::set<std::string> vegetation_labels{"tree", "plant", "palm", "flower"};  // ADE20K classes that are plants
		int update_period_ms = 500;          // model update + graph publication
		int lidar_every = 2;                 // use every Nth helios sweep (trunk confirmation + position)
		int masks_stall_timeout_ms = 5000;
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
	std::unique_ptr<rc::MaskIngestor> masks_;
	std::unique_ptr<DSR::InnerEigenAPI> inner_eigen_;
	std::unique_ptr<rc::ConceptLidarIngestor> lidar_;
	std::uint64_t sweeps_ = 0;
	std::string world_frame_;
	bool resolve_world_frame();
	bool ingest_masks(std::int64_t now_ms);     // one NEW masks frame -> both sub-models
	void ingest_lidar();
	std::vector<rc::trunk::Occluder> people() const;    // persons: occluders, and their points are not trunks
	std::vector<rc::trunk::Occluder> foliage() const;   // the canopies of this agent's own model: occluders
	std::int64_t masks_age_ms() const;

	// ── Model: the plant = trunk + canopy, composed into individuals ─────────────────────────────
	rc::trunk::TrunkMap trunks_;
	rc::veg::VegetationMap canopies_;
	rc::plants::PlantComposer composer_;
	static std::uint64_t key_of(int sub_model_id) { return static_cast<std::uint64_t>(sub_model_id) + 1; }

	// ── Publication: one `plant` node per individual ─────────────────────────────────────────────
	std::unique_ptr<DSR::RT_API> rt_api_;
	std::uint64_t world_node_id_ = 0;
	std::unordered_map<int, std::uint64_t> plant_nodes_;   // plant id -> DSR node id
	struct Published
	{
		Eigen::Vector2f c = Eigen::Vector2f::Zero();
		float trunk_var = 0.f, trunk_p = -1.f, trunk_r = 0.f, canopy_r = 0.f, height = 0.f;
		Eigen::Vector2f canopy_offset = Eigen::Vector2f::Zero();
		bool has_trunk = false, has_canopy = false;
	};
	std::unordered_map<int, Published> published_;
	std::int64_t last_update_ms_ = 0;
	void publish(const rc::plants::PlantComposer::Changes& changes);
	Published describe(const rc::plants::Plant& p) const;    // what the node should say about this plant now
	void write_plant(std::uint64_t node_id, const Published& d, bool moved);
	void delete_stale_nodes();

	// ── Diagnostics ──────────────────────────────────────────────────────────────────────────────
	std::uint64_t frames_ = 0, looks_ = 0, band_points_ = 0, veg_points_ = 0;
	std::int64_t last_status_ms_ = 0;
	void log_status();
	FPSCounter fps_;

signals:
	void presenceReady();
	void presenceLost();
};

#endif
