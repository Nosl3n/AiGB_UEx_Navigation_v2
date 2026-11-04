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
#include "../../common/vegetation/graph_layout.h"
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
	try { cfg_.close_on = configLoader.get<std::string>("Field.CloseOn"); } catch (...) {}
	try { cfg_.stable_for_ms = configLoader.get<int>("Field.StableForMs"); } catch (...) {}
	try { cfg_.update_period_ms = configLoader.get<int>("Field.UpdatePeriodMs"); } catch (...) {}
	if (cfg_.close_on == "stable")
		std::println("[field] the field is born once the plots have not changed for {:.0f} s", cfg_.stable_for_ms / 1000.0);
	else
		std::println("[field] the field is born when openfield_concept's inspection is complete (world.model_stable)");
}

void SpecificWorker::initialize()
{
	GenericWorker::initialize();
	if (not G)
	{
		qWarning() << "field_metaconcept: DSR graph not available in initialize()";
		return;
	}
	G->set_ignored_attributes<cam_rgb_att, cam_depth_att, laser_X_att, laser_Y_att, laser_Z_att>();
	rt_api_ = G->get_rt_api();
	inner_eigen_ = G->get_inner_eigen_api();

	// A metaconcept reads only the graph: there is no sensor stream to gate on, so the input is always "fresh".
	presence_protocol_.wire(
		presence_coordinator_, &statemachine, this, configLoader, G, static_cast<std::uint32_t>(agent_id),
		{
			.shutting_down      = [this] { return shutting_down_.load(); },
			.pump_primary_input = [] {},
			.primary_age_ms     = [] { return std::int64_t{0}; },
			.primary_live       = [] { return true; },
			.primary_stalled    = [](std::int64_t *age) { if (age) *age = 0; return false; },
			.emit_ready         = [this] { emit presenceReady(); },
			.emit_lost          = [this] { emit presenceLost(); },
			.compute            = [this] { compute(); },
			.terminal_shutdown  = [this] { terminal_shutdown(); },
			.on_first_operating = [this] { delete_stale_field_nodes(); },
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
	return true;
}

void SpecificWorker::compute()
{
	if (not resolve_world_frame())
		return;
	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_update_ms_ < cfg_.update_period_ms)
		return;
	last_update_ms_ = now;
	const auto plots = read_plots();
	// "Still changing": the set of plots and their rows. Any change restarts the clock.
	std::string sig;
	for (const auto &p : plots)
	{
		sig += std::format("{}:", p.node);
		if (const auto node = G->get_node(p.node); node.has_value())
			for (const auto &[key, e] : node->fano())
				if (key.second == "group_member") sig += std::format("{},", key.first);
		sig += ";";
	}
	if (sig != signature_)
	{
		signature_ = sig;
		stable_since_ms_ = now;
	}
	const bool closed = cfg_.close_on == "stable" ? now - stable_since_ms_ >= cfg_.stable_for_ms : inspection_complete();
	if (field_node_ != 0 or (not plots.empty() and closed))
		publish(plots);
	log_status(plots.size(), now);
	fps_.print("[field_metaconcept Compute]", 10000);
}

// The inspection is openfield_concept's: it raises model_stable on the world node when no standpoint left in the
// field is worth its travel (field_explorer.h). Until then the plots may still be incomplete.
bool SpecificWorker::inspection_complete() const
{
	const auto world = G->get_node(world_node_id_);
	return world.has_value() and G->get_attrib_by_name<model_stable_att>(world.value()).value_or(false);
}

// plot_metaconcept's plots: world→plot RT (centre, heading as yaw), width_m along the rows, depth_m across.
std::vector<SpecificWorker::PlotView> SpecificWorker::read_plots()
{
	std::vector<PlotView> out;
	for (const auto &n : G->get_nodes_by_type("metaconcept"))
	{
		if (not n.name().starts_with("plot_"))
			continue;
		const float len = G->get_attrib_by_name<width_m_att>(n).value_or(0.f);
		const float wid = G->get_attrib_by_name<depth_m_att>(n).value_or(0.f);
		const auto M = inner_eigen_->get_transformation_matrix(world_frame_, n.name(), 0);
		if (not M.has_value() or not (len > 0.f and wid > 0.f))
			continue;
		const auto &m = M.value();
		out.push_back({n.id(),
		               {G->get_attrib_by_name<obj_id_att>(n).value_or(-1),
		                Eigen::Vector2f(static_cast<float>(m(0, 3)), static_cast<float>(m(1, 3))),
		                static_cast<float>(std::atan2(m(1, 0), m(0, 0))), len, wid},
		               G->get_attrib_by_name<pos_x_att>(n).value_or(0.f)});
	}
	return out;
}

void SpecificWorker::publish(const std::vector<PlotView> &plots)
{
	auto world = G->get_node(world_node_id_);
	if (not world.has_value())
		return;
	std::vector<const rc::plots::RowGeom *> geoms;
	for (const auto &p : plots) geoms.push_back(&p.geom);
	const auto rect = rc::plots::enclosing(geoms);
	const float wy = G->get_attrib_by_name<pos_y_att>(world.value()).value_or(200.f);
	if (field_node_ == 0)
	{
		DSR::Node node = DSR::Node::create<metaconcept_node_type>("field_0");
		G->add_or_modify_attrib_local<obj_id_att>(node, 0);
		G->add_or_modify_attrib_local<width_m_att>(node, rect.length);
		G->add_or_modify_attrib_local<depth_m_att>(node, rect.width);
		G->add_or_modify_attrib_local<pos_x_att>(node, G->get_attrib_by_name<pos_x_att>(world.value()).value_or(200.f)
		                                                   + rc::veg::layout::kRowX0);
		G->add_or_modify_attrib_local<pos_y_att>(node, wy + rc::veg::layout::kFieldY);
		rc::provenance::stamp_creation(*G, node);
		const auto id = G->insert_node(node);
		if (not id.has_value())
			return;
		field_node_ = id.value();
		std::println("[field] created 'field_0': {} plots, {:.1f} x {:.1f} m — {}", plots.size(), rect.length, rect.width,
		             cfg_.close_on == "stable" ? std::format("no plot has changed for {:.0f} s", cfg_.stable_for_ms / 1000.0)
		                                       : std::string("openfield_concept's inspection is complete"));
	}
	// field --group_member--> plot, for exactly the current plots.
	std::set<std::uint64_t> want;
	for (const auto &p : plots) want.insert(p.node);
	for (const auto to : members_)
		if (not want.contains(to))
			G->delete_edge(field_node_, to, "group_member");
	for (const auto to : want)
		if (not members_.contains(to))
		{
			G->insert_or_assign_edge(DSR::Edge::create<group_member_edge_type>(field_node_, to));
			if (not members_.empty())
				std::println("[field] a plot joined the field ({} plots now)", want.size());
		}
	members_ = std::move(want);
	// The rectangle, rewritten only when it changed.
	if ((published_.c - rect.c).norm() >= 0.02f or std::abs(published_.length - rect.length) >= 0.05f
	    or std::abs(published_.width - rect.width) >= 0.05f or std::abs(published_.yaw - rect.yaw) >= 0.01f)
	{
		rt_api_->insert_or_assign_edge_RT(world.value(), field_node_, {rect.c.x(), rect.c.y(), 0.0f}, {0.0f, 0.0f, rect.yaw});
		if (auto node = G->get_node(field_node_); node.has_value())
		{
			G->add_or_modify_attrib_local<width_m_att>(node.value(), rect.length);
			G->add_or_modify_attrib_local<depth_m_att>(node.value(), rect.width);
			G->update_node(node.value());
		}
		published_ = rect;
	}
	// Viewer only: on the field layer, above the middle of its plots.
	if (not plots.empty())
	{
		float sx = 0.f;
		for (const auto &p : plots) sx += p.view_x;
		const std::pair<float, float> xy{sx / static_cast<float>(plots.size()), wy + rc::veg::layout::kFieldY};
		if (xy != laid_out_)
			if (auto node = G->get_node(field_node_); node.has_value())
			{
				G->add_or_modify_attrib_local<pos_x_att>(node.value(), xy.first);
				G->add_or_modify_attrib_local<pos_y_att>(node.value(), xy.second);
				if (G->update_node(node.value()))
					laid_out_ = xy;
			}
	}
}

void SpecificWorker::delete_stale_field_nodes()
{
	int n = 0;
	for (const auto &node : G->get_nodes_by_type("metaconcept"))
		if (node.name().starts_with("field_") and node.id() != field_node_)
		{
			G->delete_node(node.id());
			++n;
		}
	if (n > 0)
		std::println("[field] removed {} stale field node(s) left by a previous run", n);
}

void SpecificWorker::log_status(std::size_t plots, std::int64_t now)
{
	if (now - last_status_ms_ < 5000)
		return;
	last_status_ms_ = now;
	if (field_node_ != 0)
		std::println("[field] status: field_0 groups {} plots ({:.1f} x {:.1f} m)", plots, published_.length, published_.width);
	else
	{
		if (cfg_.close_on == "stable")
			std::println("[field] status: {} plots, unchanged for {:.0f} / {:.0f} s before the field is born", plots,
			             (now - stable_since_ms_) / 1000.0, cfg_.stable_for_ms / 1000.0);
		else
			std::println("[field] status: {} plots, waiting for the inspection to complete", plots);
	}
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
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}
