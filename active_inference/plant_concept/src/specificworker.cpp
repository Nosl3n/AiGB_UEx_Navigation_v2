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
#include "specificworker.h"

#include "../../common/agent_exit/terminal_exit.h"
#include "../../common/graph_provenance/creation_stamp.h"
#include "../../common/stream_gate/stream_gate.h"
#include "../../common/vegetation/graph_layout.h"
#include "../../common/vegetation/plant_discs.h"
#include "../../common/world_frame/world_frame.h"

#include <QCoreApplication>
#include <QDateTime>
#include <algorithm>
#include <cmath>
#include <format>
#include <print>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check)
    : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if (this->startup_check_flag)
	{
		this->startup_check();
		return;
	}

	load_config();

	// The three model pieces are pure Eigen: prove each on synthetic data before trusting it with the field.
	for (const auto &[name, test] : {std::pair{"trunk", &rc::trunk::TrunkMap::self_test},
	                                 std::pair{"canopy", &rc::veg::VegetationMap::self_test},
	                                 std::pair{"composer", &rc::plants::PlantComposer::self_test}})
		if (std::string report; test(&report))
			std::println("{}", report);
		else
			qCritical("[plants] %s self-test FAILED:\n%s", name, report.c_str());
	trunks_.set_params(cfg_.trunk);
	canopies_.set_params(cfg_.canopy);

	// Agent-presence state machine: Waiting -> Operating -> Degraded (same shape as every concept agent).
	const int period = configLoader.get<int>("Period.Compute");
	states["Waiting"] = std::make_unique<GRAFCETStep>("Waiting", period,
		std::bind(&SpecificWorker::waiting_loop, this), std::bind(&SpecificWorker::waiting_enter, this));
	states["Operating"] = std::make_unique<GRAFCETStep>("Operating", period,
		std::bind(&SpecificWorker::operating_loop, this), std::bind(&SpecificWorker::operating_enter, this));
	states["Degraded"] = std::make_unique<GRAFCETStep>("Degraded", period,
		std::bind(&SpecificWorker::degraded_loop, this), std::bind(&SpecificWorker::degraded_enter, this));

	states["Compute"]->addTransition(states["Compute"].get(), SIGNAL(entered()), states["Waiting"].get());
	states["Waiting"]->addTransition(this, SIGNAL(presenceReady()), states["Operating"].get());
	states["Operating"]->addTransition(this, SIGNAL(presenceLost()), states["Degraded"].get());
	states["Degraded"]->addTransition(states["Degraded"].get(), SIGNAL(entered()), states["Waiting"].get());

	statemachine.addState(states["Waiting"].get());
	statemachine.addState(states["Operating"].get());
	statemachine.addState(states["Degraded"].get());
	statemachine.setChildMode(QState::ExclusiveStates);
	statemachine.start();
	if (auto error = statemachine.errorString(); error.length() > 0)
	{
		qWarning() << error;
		throw error;
	}
}

SpecificWorker::~SpecificWorker()
{
	request_shutdown();
}

void SpecificWorker::load_config()
{
	const auto opt = [this](const std::string &key, auto &out)
	{
		try { out = configLoader.get<std::remove_reference_t<decltype(out)>>(key); } catch (...) {}
	};
	const auto optf = [&](const std::string &key, float &out)
	{
		double v = out;
		opt(key, v);
		out = static_cast<float>(v);
	};
	// The trunk sub-model.
	auto &m = cfg_.trunk;
	optf("Trunk.KeepoutRadiusM", cfg_.keepout_radius_m);
	optf("Trunk.PersonRadiusM", cfg_.person_radius_m);
	optf("Trunk.BandZMinM", m.band_z_min);
	optf("Trunk.BandZMaxM", m.band_z_max);
	optf("Trunk.ClusterRadiusM", m.cluster_r);
	optf("Trunk.BirthMass", m.birth_mass);
	optf("Trunk.LlrHit", m.llr_hit);
	optf("Trunk.LlrMiss", m.llr_miss);
	optf("Trunk.LookPeriodMs", m.look_period_ms);
	opt("Trunk.LidarConfirmPoints", m.lidar_confirm_pts);
	optf("Trunk.AssocGateM", m.assoc_gate_m);
	optf("Trunk.MinSeparationM", m.min_separation_m);
	optf("Trunk.RemovalProb", m.removal_prob);
	optf("Trunk.ZedMaxRangeM", m.zed_max_range);
	optf("Trunk.ZedMinRangeM", m.zed_min_range);
	optf("Trunk.LidarConfirmDzM", m.lidar_confirm_dz);
	optf("Trunk.ZedHalfFovRad", m.zed_half_fov);
	opt("Trunk.LidarEvery", cfg_.lidar_every);
	opt("Ablation.LidarConfirm", m.lidar_confirm);
	opt("Ablation.VisibilityModel", m.visibility_model);
	if (not m.lidar_confirm or not m.visibility_model)
		std::println("[plants] ABLATION: LidarConfirm={} VisibilityModel={}", m.lidar_confirm, m.visibility_model);
	std::vector<std::string> labels;
	opt("Trunk.TrunkLabels", labels);
	if (not labels.empty()) cfg_.trunk_labels = {labels.begin(), labels.end()};
	// The canopy sub-model.
	auto &v = cfg_.canopy;
	optf("Canopy.RadiusM", v.plant_radius_m);
	optf("Canopy.CellM", v.cell_m);
	optf("Canopy.ZMinM", v.z_min);
	optf("Canopy.ZMaxM", v.z_max);
	optf("Canopy.BirthMass", v.birth_mass);
	optf("Canopy.LookPeriodMs", v.look_period_ms);
	labels.clear();
	opt("Canopy.Labels", labels);
	if (not labels.empty()) cfg_.vegetation_labels = {labels.begin(), labels.end()};
	// The agent.
	opt("Plants.UpdatePeriodMs", cfg_.update_period_ms);
	opt("Plants.MasksStallTimeoutMs", cfg_.masks_stall_timeout_ms);

	std::println("[plants] trunk: keep-out r={:.2f} m, band z [{:.2f}, {:.2f}] m, birth {:.0f} cell-frames + {} LiDAR "
	             "returns | canopy: r={:.2f} m, birth {:.0f} cell-frames | a trunk inside a canopy is that plant's",
	             cfg_.keepout_radius_m, m.band_z_min, m.band_z_max, m.birth_mass, m.lidar_confirm_pts,
	             v.plant_radius_m, v.birth_mass);
}

void SpecificWorker::initialize()
{
	GenericWorker::initialize();
	if (not G)
	{
		qWarning() << "plant_concept: DSR graph not available in initialize()";
		return;
	}
	G->set_ignored_attributes<cam_rgb_att, cam_depth_att, laser_X_att, laser_Y_att, laser_Z_att>();

	rt_api_ = G->get_rt_api();
	inner_eigen_ = G->get_inner_eigen_api();
	masks_ = std::make_unique<rc::MaskIngestor>(G);
	// The helios subscriber comes up lazily inside pump(), once the node and its media descriptor exist.
	lidar_ = std::make_unique<rc::ConceptLidarIngestor>(G, inner_eigen_.get(),
	                                                    [] { return rc::LidarGates{.helios_precision = 1.0f}; });

	presence_protocol_.wire(
		presence_coordinator_, &statemachine, this, configLoader, G, static_cast<std::uint32_t>(agent_id),
		{
			.shutting_down      = [this] { return shutting_down_.load(); },
			// The masks node is POLLED, so it is pumped while Waiting. The world frame first: MaskIngestor drops (and
			// does not count as fresh) a frame it cannot re-frame, so with its default "room" target in a field it
			// would never read one and this agent never be admitted.
			.pump_primary_input = [this] { if (masks_ and resolve_world_frame()) masks_->refresh(); },
			.primary_age_ms     = [this] { return masks_age_ms(); },
			.primary_live       = [this] { return rc::stream::live(masks_age_ms(), cfg_.masks_stall_timeout_ms); },
			.primary_stalled    = [this](std::int64_t *age)
			{
				const auto a = masks_age_ms();
				if (age) *age = a;
				return rc::stream::stalled(a, cfg_.masks_stall_timeout_ms, presence_protocol_.operating_since_ms(),
				                           QDateTime::currentMSecsSinceEpoch());
			},
			.emit_ready         = [this] { emit presenceReady(); },
			.emit_lost          = [this] { emit presenceLost(); },
			.compute            = [this] { compute(); },
			.terminal_shutdown  = [this] { terminal_shutdown(); },
			.on_first_operating = [this] { delete_stale_nodes(); },
			.on_optional_peer_lost  = [](const std::string &name, std::uint32_t)
			                          { qInfo() << "[Presence] optional peer lost:" << QString::fromStdString(name); },
			.on_optional_peer_ready = [](const std::string &name, std::uint32_t)
			                          { qInfo() << "[Presence] optional peer ready:" << QString::fromStdString(name); },
		});

	QObject::connect(QCoreApplication::instance(), &QCoreApplication::aboutToQuit,
	                 this, &SpecificWorker::terminal_shutdown, Qt::UniqueConnection);
}

bool SpecificWorker::resolve_world_frame()
{
	if (world_node_id_ != 0)
		return true;
	const auto frame = rc::world::frame_node(*G);
	if (not frame.has_value())
		return false;
	world_node_id_ = frame->id();
	world_frame_ = frame->name();
	masks_->set_frames("zed", world_frame_);
	std::println("[plants] world frame '{}': plants are placed in it", world_frame_);
	return true;
}

void SpecificWorker::compute()
{
	if (not resolve_world_frame())
		return;
	const auto now = QDateTime::currentMSecsSinceEpoch();
	ingest_masks(now);
	ingest_lidar();
	if (now - last_update_ms_ >= cfg_.update_period_ms)
	{
		last_update_ms_ = now;
		canopies_.update();
		trunks_.update(now);
		std::vector<rc::plants::Part> t, c;
		for (const auto &tr : trunks_.trunks())
			t.push_back({key_of(tr.id), tr.c, 0.0f});
		for (const auto &cp : canopies_.plants())
			c.push_back({key_of(cp.id), cp.c, cfg_.canopy.plant_radius_m});
		publish(composer_.update(t, c));
	}
	log_status();
	fps_.print("[plant_concept Compute]", 5000);
}

// ── ZED: one masks frame feeds both parts — the 'tree' band points to the trunk model, every vegetation point to
//    the canopy model — with the camera's pose and whether this frame was a real look ──────────────────────
bool SpecificWorker::ingest_masks(std::int64_t now_ms)
{
	if (not masks_->refresh())
		return false;
	++frames_;
	const auto &pk = masks_->packet();
	std::vector<Eigen::Vector3f> band, veg;
	bool looked = false;
	const auto &p = trunks_.params();
	for (const auto &s : pk.slices)
	{
		// Only the front RGB-D camera's semantic (ADE20K) masks: a YOLO-seg instance mask is never a plant here
		// (COCO's "potted plant" is a pot on a table), and a bearing-only slice has no 3D.
		if (not s.is_semantic() or not s.may_fit_geometry() or not cfg_.vegetation_labels.contains(s.label))
			continue;
		// Semantic masks only exist on the frames the ADE20K model actually ran (it is decimated). A frame with
		// vegetation in it is a real look: a trunk expected in view and absent from it is a genuine miss.
		looked = true;
		if (s.support_end > pk.support_points.size() or s.support_begin >= s.support_end)
			continue;
		const bool trunk_class = cfg_.trunk_labels.contains(s.label);
		for (std::size_t i = s.support_begin; i < s.support_end; ++i)
		{
			const auto &q = pk.support_points[i];
			veg.push_back(q);
			if (trunk_class and q.z() >= p.band_z_min and q.z() <= p.band_z_max)
				band.push_back(q);
		}
	}
	if (not veg.empty())
	{
		veg_points_ += veg.size();
		canopies_.add_frame(veg, now_ms);
	}
	if (not looked and band.empty())
		return true;
	const auto T = inner_eigen_->get_transformation_matrix(world_frame_, "zed", 0);
	if (not T.has_value())
		return true;
	const auto &M = T.value();
	const Eigen::Vector2f cam(static_cast<float>(M(0, 3)), static_cast<float>(M(1, 3)));
	// The ZED's optical axis is its +Y (x right, y forward/depth, z up — the retina deprojection convention).
	const float heading = static_cast<float>(std::atan2(M(1, 1), M(0, 1)));
	looks_ += looked ? 1 : 0;
	band_points_ += band.size();
	const auto ppl = people();
	auto occ = foliage();
	occ.insert(occ.end(), ppl.begin(), ppl.end());
	trunks_.add_zed_frame(band, cam, heading, occ, ppl, looked, now_ms);
	return true;
}

void SpecificWorker::ingest_lidar()
{
	if (not lidar_ or not lidar_->pump() or not lidar_->helios_fresh())
		return;
	if ((sweeps_++ % static_cast<std::uint64_t>(std::max(1, cfg_.lidar_every))) != 0)
		return;
	trunks_.add_lidar_sweep(lidar_->sweep_room(), lidar_->origin_room(), people());
}

// What can hide a trunk from the camera: the plants' own canopies (this agent's canopy model) ...
std::vector<rc::trunk::Occluder> SpecificWorker::foliage() const
{
	std::vector<rc::trunk::Occluder> occ;
	for (const auto &c : canopies_.plants())
		occ.push_back({c.c, cfg_.canopy.plant_radius_m});
	return occ;
}

// ... and people, whose legs also cross the trunk band: their points are never a trunk's, for either sensor.
std::vector<rc::trunk::Occluder> SpecificWorker::people() const
{
	std::vector<rc::trunk::Occluder> occ;
	for (const auto &n : G->get_nodes_by_type("person"))
		if (const auto t = inner_eigen_->transform(world_frame_, n.name(), 0); t.has_value() and t->allFinite())
			occ.push_back({Eigen::Vector2f(static_cast<float>(t->x()), static_cast<float>(t->y())), cfg_.person_radius_m});
	return occ;
}

std::int64_t SpecificWorker::masks_age_ms() const
{
	return masks_ ? masks_->ms_since_last_frame() : -1;
}

// ── Graph: one `plant` node per individual, its parts as attributes (conceptos.md) ────────────────────────
SpecificWorker::Published SpecificWorker::describe(const rc::plants::Plant &p) const
{
	Published d;
	d.c = p.c;
	if (p.trunk != 0)
		if (const auto it = std::ranges::find(trunks_.trunks(), p.trunk, [](const auto &t) { return key_of(t.id); });
		    it != trunks_.trunks().end())
		{
			d.has_trunk = true;
			d.trunk_var = it->var;
			d.trunk_p = it->existence.p_exists();
			d.trunk_r = it->radius_n > 0 ? it->radius : 0.0f;   // measured by the LiDAR; 0 = not yet
		}
	if (p.canopy != 0)
		if (const auto it = std::ranges::find(canopies_.plants(), p.canopy, [](const auto &c) { return key_of(c.id); });
		    it != canopies_.plants().end())
		{
			d.has_canopy = true;
			d.canopy_r = cfg_.canopy.plant_radius_m;
			d.canopy_offset = it->c - p.c;
			d.height = it->height;
			if (not d.has_trunk)
				d.trunk_var = it->sigma_xy * it->sigma_xy;   // the plant stands on its canopy centre
		}
	return d;
}

void SpecificWorker::publish(const rc::plants::PlantComposer::Changes &changes)
{
	for (const int id : changes.removed)
		if (const auto it = plant_nodes_.find(id); it != plant_nodes_.end())
		{
			G->delete_node(it->second);
			std::println("[plants] plant_{} removed: its trunk was verified gone and it has no canopy", id);
			plant_nodes_.erase(it);
			published_.erase(id);
		}
	auto world = G->get_node(world_node_id_);
	if (not world.has_value())
		return;
	for (const auto &[id, p] : composer_.plants())
	{
		const Published d = describe(p);
		auto it = plant_nodes_.find(id);
		if (it == plant_nodes_.end())
		{
			const std::string name = std::format("{}{}", rc::veg::kPlantPrefix, id);
			DSR::Node node = DSR::Node::create<plant_node_type>(name);
			G->add_or_modify_attrib_local<obj_id_att>(node, id);
			const float wx = G->get_attrib_by_name<pos_x_att>(world.value()).value_or(200.f);
			const float wy = G->get_attrib_by_name<pos_y_att>(world.value()).value_or(200.f);
			// Born in no row: the viewer's unassigned zone; row_metaconcept moves it under its row.
			const auto [px, py] = rc::veg::layout::unassigned_plant(wx, wy, id);
			G->add_or_modify_attrib_local<pos_x_att>(node, px);
			G->add_or_modify_attrib_local<pos_y_att>(node, py);
			rc::provenance::stamp_creation(*G, node);
			const auto nid = G->insert_node(node);
			if (not nid.has_value())
			{
				qWarning() << "plant_concept: failed to insert" << name.c_str();
				continue;
			}
			it = plant_nodes_.emplace(id, nid.value()).first;
			std::println("[plants] created '{}' at ({:.2f}, {:.2f}): {}", name, p.c.x(), p.c.y(),
			             d.has_trunk and d.has_canopy ? "trunk + canopy" : d.has_trunk ? "trunk (canopy unseen)"
			                                                                           : "canopy (trunk unseen)");
			write_plant(it->second, d, true);
			published_[id] = d;
			continue;
		}
		// Rewrite only what changed: a trunk settles a few cm, its belief drifts, a canopy appears or grows.
		const auto pr = published_.find(id);
		const bool moved = pr == published_.end() or (pr->second.c - d.c).norm() >= 0.01f;
		const bool changed = pr == published_.end() or pr->second.has_trunk != d.has_trunk
		                     or pr->second.has_canopy != d.has_canopy or std::abs(pr->second.trunk_p - d.trunk_p) >= 0.05f
		                     or (pr->second.canopy_offset - d.canopy_offset).norm() >= 0.02f
		                     or std::abs(pr->second.height - d.height) >= 0.05f
		                     or std::abs(pr->second.trunk_r - d.trunk_r) >= 0.002f;
		if (not moved and not changed)
			continue;
		if (pr != published_.end() and (pr->second.has_trunk != d.has_trunk or pr->second.has_canopy != d.has_canopy))
			std::println("[plants] plant_{} now has {}", id,
			             d.has_trunk and d.has_canopy ? "trunk + canopy" : d.has_trunk ? "only its trunk" : "only its canopy");
		write_plant(it->second, d, moved);
		published_[id] = d;
	}
}

// The node's attributes (conceptos.md, `plant`) and, if it moved, its world->plant RT with the position covariance.
void SpecificWorker::write_plant(std::uint64_t node_id, const Published &d, bool moved)
{
	if (moved)
		if (auto world = G->get_node(world_node_id_); world.has_value())
		{
			rt_api_->insert_or_assign_edge_RT(world.value(), node_id, {d.c.x(), d.c.y(), 0.0f}, {0.0f, 0.0f, 0.0f});
			if (auto edge = G->get_edge(world_node_id_, node_id, "RT"); edge.has_value())
			{
				std::vector<float> cov(36, 0.0f);
				cov[0] = cov[7] = std::max(d.trunk_var, 1e-4f);
				cov[14] = 0.01f;
				for (int k = 3; k < 6; ++k) cov[k * 6 + k] = 1.0f;
				G->add_or_modify_attrib_local<rt_covariance_att>(edge.value(), cov);
				G->insert_or_assign_edge(edge.value());
			}
		}
	auto node = G->get_node(node_id);
	if (not node.has_value())
		return;
	auto &n = node.value();
	G->add_or_modify_attrib_local<plant_has_trunk_att>(n, d.has_trunk);
	G->add_or_modify_attrib_local<plant_trunk_exist_prob_att>(n, d.has_trunk ? d.trunk_p : 0.0f);
	G->add_or_modify_attrib_local<plant_keepout_radius_m_att>(n, d.has_trunk ? cfg_.keepout_radius_m : 0.0f);
	G->add_or_modify_attrib_local<plant_trunk_radius_m_att>(n, d.has_trunk ? d.trunk_r : 0.0f);
	G->add_or_modify_attrib_local<plant_has_canopy_att>(n, d.has_canopy);
	G->add_or_modify_attrib_local<plant_canopy_radius_m_att>(n, d.has_canopy ? d.canopy_r : 0.0f);
	G->add_or_modify_attrib_local<plant_canopy_offset_att>(n, std::vector<float>{d.canopy_offset.x(), d.canopy_offset.y()});
	G->add_or_modify_attrib_local<plant_height_m_att>(n, d.height);
	// The individual exists as long as one of its parts does: a seen canopy (which the canopy model never retracts),
	// else its trunk's existence belief.
	G->add_or_modify_attrib_local<obj_exist_prob_att>(n, d.has_canopy ? 1.0f : d.trunk_p);
	G->update_node(n);
}

// Plants left by a run that did not exit gracefully, and the trunk_* / plant_* nodes of the two agents this one
// replaced (trunk_concept, vegetation_concept), if a previous run of theirs was killed.
void SpecificWorker::delete_stale_nodes()
{
	int n = 0;
	for (const std::string type : {"plant", "trunk"})
		for (const auto &node : G->get_nodes_by_type(type))
			if ((node.name().starts_with(rc::veg::kPlantPrefix) or node.name().starts_with("trunk_"))
			    and std::ranges::none_of(plant_nodes_, [&](const auto &kv) { return kv.second == node.id(); }))
			{
				G->delete_node(node.id());
				++n;
			}
	if (n > 0)
		std::println("[plants] removed {} stale node(s) left by a previous run", n);
}

void SpecificWorker::log_status()
{
	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_status_ms_ < 5000)
		return;
	last_status_ms_ = now;
	int complete = 0, trunk_only = 0, canopy_only = 0;
	for (const auto &[id, p] : composer_.plants())
	{
		if (p.trunk != 0 and p.canopy != 0) ++complete;
		else if (p.trunk != 0) ++trunk_only;
		else ++canopy_only;
	}
	std::println("[plants] status: {} plants ({} complete, {} trunk only, {} canopy only) | {} trunks ({} pending), "
	             "{} canopies | {} frames, {} looks", composer_.plants().size(), complete, trunk_only, canopy_only,
	             trunks_.trunks().size(), trunks_.pending(), canopies_.plants().size(), frames_, looks_);
}

void SpecificWorker::request_shutdown()
{
	if (shutting_down_.exchange(true)) return;
	if (G) disconnect(G.get(), nullptr, this, nullptr);
	if (not owned_nodes_cleaned_)
	{
		owned_nodes_cleaned_ = true;
		presence_coordinator_.cleanup_owned_nodes();
	}
}

void SpecificWorker::terminal_shutdown()
{
	static std::atomic<bool> terminating{false};
	if (terminating.exchange(true)) return;
	rc::agent::terminal_exit([this] { request_shutdown(); }, [this] { if (G) G->reset(); });
}

void SpecificWorker::emergency()
{
	std::cout << "Emergency worker" << std::endl;
}

void SpecificWorker::restore()
{
	std::cout << "Restore worker" << std::endl;
}

int SpecificWorker::startup_check()
{
	std::string report;
	bool ok = rc::trunk::TrunkMap::self_test(&report);
	ok = rc::veg::VegetationMap::self_test(&report) and ok;
	ok = rc::plants::PlantComposer::self_test(&report) and ok;
	std::println("{}", report);
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return ok ? 0 : 1;
}
