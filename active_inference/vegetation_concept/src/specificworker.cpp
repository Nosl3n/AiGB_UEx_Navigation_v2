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
#include "../../common/world_frame/world_frame.h"
#include "../../common/vegetation/graph_layout.h"
#include "../../common/vegetation/plant_discs.h"

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

	// The map is pure Eigen: prove it on a synthetic orchard before trusting it with the field.
	if (std::string report; rc::veg::VegetationMap::self_test(&report))
		std::println("{}", report);
	else
		qCritical("%s", report.c_str());
	map_.set_params(cfg_.map);

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
	auto &m = cfg_.map;
	optf("Vegetation.PlantRadiusM", m.plant_radius_m);
	optf("Vegetation.CellM", m.cell_m);
	optf("Vegetation.ZMinM", m.z_min);
	optf("Vegetation.ZMaxM", m.z_max);
	optf("Vegetation.BirthMass", m.birth_mass);
	optf("Vegetation.LookPeriodMs", m.look_period_ms);
	std::vector<std::string> labels;
	opt("Vegetation.Labels", labels);
	if (not labels.empty())
		cfg_.labels = {labels.begin(), labels.end()};
	opt("Vegetation.UpdatePeriodMs", cfg_.update_period_ms);
	opt("Vegetation.MasksStallTimeoutMs", cfg_.masks_stall_timeout_ms);

	std::string list;
	for (const auto &l : cfg_.labels) list += (list.empty() ? "" : ", ") + l;
	std::println("[vegetation] plant radius {:.2f} m, birth mass {:.0f} cell-frames, z [{:.2f}, {:.2f}] m, classes: {}",
	             m.plant_radius_m, m.birth_mass, m.z_min, m.z_max, list);
}

void SpecificWorker::initialize()
{
	GenericWorker::initialize();
	if (not G)
	{
		qWarning() << "vegetation_concept: DSR graph not available in initialize()";
		return;
	}
	// Reads only the 'masks' node: drop the payload blobs peers publish.
	G->set_ignored_attributes<cam_rgb_att, cam_depth_att, laser_X_att, laser_Y_att, laser_Z_att>();

	rt_api_ = G->get_rt_api();
	masks_ = std::make_unique<rc::MaskIngestor>(G);

	presence_protocol_.wire(
		presence_coordinator_, &statemachine, this, configLoader, G, static_cast<std::uint32_t>(agent_id),
		{
			.shutting_down      = [this] { return shutting_down_.load(); },
			// The masks node is POLLED, so it must be pumped while Waiting or its age never advances. The world
			// frame first: MaskIngestor drops (and does not count as fresh) a frame it cannot re-frame, so with
			// its default "room" target in a field it would never read a frame and this agent never be admitted.
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
			// Plants left by a run that did not exit gracefully would be duplicated by this run's own.
			.on_first_operating = [this] { delete_stale_plant_nodes(); },
			.on_optional_peer_lost  = [](const std::string &name, std::uint32_t)
			                          { qInfo() << "[Presence] optional peer lost:" << QString::fromStdString(name); },
			.on_optional_peer_ready = [](const std::string &name, std::uint32_t)
			                          { qInfo() << "[Presence] optional peer ready:" << QString::fromStdString(name); },
		});

	QObject::connect(QCoreApplication::instance(), &QCoreApplication::aboutToQuit,
	                 this, &SpecificWorker::terminal_shutdown, Qt::UniqueConnection);
}

// ── One Operating tick: ingest the newest masks frame; extract and publish the plants at a bounded rate ──
bool SpecificWorker::resolve_world_frame()
{
	if (world_node_id_ != 0)
		return true;
	const auto frame = rc::world::frame_node(*G);
	if (not frame.has_value())
		return false;                     // no localization yet: nothing can be placed
	world_node_id_ = frame->id();
	world_frame_ = frame->name();
	masks_->set_frames("zed", world_frame_);
	std::println("[vegetation] world frame '{}': plants are placed in it", world_frame_);
	return true;
}

void SpecificWorker::compute()
{
	if (not resolve_world_frame())
		return;

	ingest_masks();

	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_update_ms_ >= cfg_.update_period_ms)
	{
		last_update_ms_ = now;
		publish_plants(map_.update());
	}
	log_status();
	fps_.print("[vegetation_concept Compute]", 5000);
}

bool SpecificWorker::ingest_masks()
{
	if (not masks_->refresh())
		return false;
	++frames_;
	const auto &pk = masks_->packet();
	std::vector<Eigen::Vector3f> pts;
	for (const auto &s : pk.slices)
	{
		// Only the FRONT RGB-D camera's semantic (ADE20K) masks of a plant class: a YOLO-seg instance mask is
		// never a plant here (COCO's "potted plant" is a pot on a table), and a bearing-only slice has no 3D.
		if (not s.is_semantic() or not s.may_fit_geometry() or not cfg_.labels.contains(s.label))
			continue;
		if (s.support_end > pk.support_points.size() or s.support_begin >= s.support_end)
			continue;
		pts.insert(pts.end(), pk.support_points.begin() + static_cast<std::ptrdiff_t>(s.support_begin),
		           pk.support_points.begin() + static_cast<std::ptrdiff_t>(s.support_end));
	}
	if (pts.empty())
		return true;
	++frames_with_veg_;
	points_ += pts.size();
	map_.add_frame(pts, QDateTime::currentMSecsSinceEpoch());
	return true;
}

std::int64_t SpecificWorker::masks_age_ms() const
{
	return masks_ ? masks_->ms_since_last_frame() : -1;
}

// ── Graph ─────────────────────────────────────────────────────────────────────────────────────────
void SpecificWorker::publish_plants(const rc::veg::VegetationMap::Changes &changes)
{
	// Duplicates the map merged away (the older plant kept the space).
	for (const int id : changes.removed)
	{
		delete_plant_node(id);
		std::println("[vegetation] canopy_{} removed: it overlapped an older canopy", id);
	}

	for (const auto &p : map_.plants())
	{
		if (const auto it = plant_nodes_.find(p.id); it == plant_nodes_.end())
			insert_plant_node(p);
		else
			update_plant_node(p, it->second);
	}
}

void SpecificWorker::delete_plant_node(int plant_id)
{
	if (const auto it = plant_nodes_.find(plant_id); it != plant_nodes_.end())
	{
		G->delete_node(it->second);
		plant_nodes_.erase(it);
		published_.erase(plant_id);
	}
}

void SpecificWorker::insert_plant_node(const rc::veg::Plant &p)
{
	auto world = G->get_node(world_node_id_);
	if (not world.has_value())
		return;
	const std::string name = std::format("{}{}", rc::veg::kCanopyPrefix, p.id);
	DSR::Node node = DSR::Node::create<canopy_node_type>(name);
	const float diameter = 2.0f * cfg_.map.plant_radius_m;
	G->add_or_modify_attrib_local<obj_id_att>(node, p.id);
	G->add_or_modify_attrib_local<obj_width_m_att>(node, diameter);
	G->add_or_modify_attrib_local<width_m_att>(node, diameter);    // residual_concept's explainer box
	G->add_or_modify_attrib_local<depth_m_att>(node, diameter);
	G->add_or_modify_attrib_local<height_m_att>(node, std::max(p.height, 0.1f));
	G->add_or_modify_attrib_local<obj_exist_prob_att>(node, 1.0f);
	{
		// Graph-viewer layout only: born in no plant, in the "unassigned" strip (12 px below the trunks' slots);
		// plant_concept puts it next to its plant.
		const float wx = G->get_attrib_by_name<pos_x_att>(world.value()).value_or(200.f);
		const float wy = G->get_attrib_by_name<pos_y_att>(world.value()).value_or(200.f);
		const auto [px, py] = rc::veg::layout::unassigned_part(wx, wy + 12.f, p.id);
		G->add_or_modify_attrib_local<pos_x_att>(node, px);
		G->add_or_modify_attrib_local<pos_y_att>(node, py);
	}
	rc::provenance::stamp_creation(*G, node);
	const auto id = G->insert_node(node);
	if (not id.has_value())
	{
		qWarning() << "vegetation_concept: failed to insert" << name.c_str();
		return;
	}
	plant_nodes_[p.id] = id.value();
	std::println("[vegetation] created '{}' at {} ({:.2f}, {:.2f}) r={:.2f} m top={:.2f} m (evidence {:.0f})",
	             name, world_frame_, p.c.x(), p.c.y(), cfg_.map.plant_radius_m, p.height, p.mass);
	update_plant_node(p, id.value());
}

void SpecificWorker::update_plant_node(const rc::veg::Plant &p, std::uint64_t node_id)
{
	// Plants do not move; only the ESTIMATE of their centre settles. Rewrite the RT when it moved > 1 cm.
	if (const auto it = published_.find(p.id);
	    it != published_.end() and (it->second.c - p.c).norm() < 0.01f and std::abs(it->second.height - p.height) < 0.05f)
		return;
	auto world = G->get_node(world_node_id_);
	auto node = G->get_node(node_id);
	if (not world.has_value() or not node.has_value())
		return;
	rt_api_->insert_or_assign_edge_RT(world.value(), node_id, {p.c.x(), p.c.y(), 0.0f}, {0.0f, 0.0f, 0.0f});
	// The centre's own uncertainty on the RT edge, for consumers that weigh the disc by it (residual_concept).
	if (auto edge = G->get_edge(world_node_id_, node_id, "RT"); edge.has_value())
	{
		const float var = std::max(p.sigma_xy * p.sigma_xy, 1e-4f);
		std::vector<float> cov(36, 0.0f);
		cov[0] = cov[7] = var;           // x, y
		cov[14] = 0.01f;                 // z: on the ground
		for (int d = 3; d < 6; ++d) cov[d * 6 + d] = 1.0f;   // orientation: meaningless for a disc
		G->add_or_modify_attrib_local<rt_covariance_att>(edge.value(), cov);
		G->insert_or_assign_edge(edge.value());
	}
	G->add_or_modify_attrib_local<height_m_att>(node.value(), std::max(p.height, 0.1f));
	G->update_node(node.value());
	published_[p.id] = {p.c, p.height};
}

void SpecificWorker::delete_stale_plant_nodes()
{
	int n = 0;
	for (const auto &node : G->get_nodes_by_type(std::string(rc::veg::kCanopyType)))
		if (node.name().starts_with(rc::veg::kCanopyPrefix)
		    and std::ranges::none_of(plant_nodes_, [&](const auto &kv) { return kv.second == node.id(); }))
		{
			G->delete_node(node.id());
			++n;
		}
	if (n > 0)
		std::println("[vegetation] removed {} stale plant node(s) left by a previous run", n);
}

void SpecificWorker::log_status()
{
	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_status_ms_ < 5000)
		return;
	last_status_ms_ = now;
	std::println("[vegetation] status: masks frames {} (with vegetation {}), vegetation points {}, grid cells {}, plants {}",
	             frames_, frames_with_veg_, points_, map_.cells(), map_.plants().size());
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────────────────────────
void SpecificWorker::request_shutdown()
{
	if (shutting_down_.exchange(true)) return;
	if (G) disconnect(G.get(), nullptr, this, nullptr);
	masks_.reset();
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
	const bool ok = rc::veg::VegetationMap::self_test(&report);
	std::println("{}", report);
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return ok ? 0 : 1;
}
