/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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

#include <algorithm>
#include <array>
#include <span>
#include <cmath>
#include <cstdlib>
#include <format>
#include <print>
#include <vector>

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

namespace
{
using Mat3 = std::array<double, 9>;   // row-major
using Vec3 = std::array<double, 3>;

Mat3 from_webots(const double* m) { Mat3 r; for (int i = 0; i < 9; ++i) r[i] = m[i]; return r; }
Mat3 mul(const Mat3& a, const Mat3& b)
{
	Mat3 r{};
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			for (int k = 0; k < 3; ++k)
				r[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
	return r;
}
Mat3 transpose(const Mat3& a) { return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]}; }
Vec3 mul(const Mat3& a, const Vec3& v)
{
	return {a[0] * v[0] + a[1] * v[1] + a[2] * v[2], a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
	        a[6] * v[0] + a[7] * v[1] + a[8] * v[2]};
}
Vec3 sub(const double* a, const double* b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

// Webots devices are FLU (+X forward/optical axis, +Y left). The robot convention used in the graph is
// +Y forward/optical axis, +X right. A frame of the second kind, expressed in the first: Rz(-90 deg).
const Mat3 kFluFromRobot = {0, 1, 0, -1, 0, 0, 0, 0, 1};

// Euler angles for cortex's convention R = Rx(a) * Ry(b) * Rz(c).
Vec3 euler_xyz(const Mat3& R)
{
	const double b = std::asin(std::clamp(R[2], -1.0, 1.0));
	return {std::atan2(-R[5], R[8]), b, std::atan2(-R[1], R[0])};
}

std::uint64_t wall_ms()
{
	return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count());
}
}  // namespace

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if (this->startup_check_flag)
		this->startup_check();
	else
	{
		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();
		auto error = statemachine.errorString();
		if (error.length() > 0)
		{
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	stop_ = true;
	pilot_.stop();
	if (sim_thread_.joinable())
	{
		// Still waiting for Webots to accept the connection (no drone in the world): the constructor of
		// webots::Supervisor blocks inside libController and cannot be interrupted, so do not join it.
		if (connected_) sim_thread_.join();
		else sim_thread_.detach();
	}
	std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize()
{
	GenericWorker::initialize();
	auto opt = [this]<class T>(const char* key, T& out) {
		try { out = configLoader.get<T>(key); } catch (...) {}
	};
	opt("Webots.Url", params.webots_url);
	opt("Webots.CarrierDef", params.carrier_def);
	opt("Camera.Name", params.camera_name);
	opt("Camera.PeriodMs", params.camera_period_ms);
	opt("Camera.Pitch", params.camera_pitch);
	opt("Joystick.ModeId", params.mode_id);
	opt("Joystick.MaxHorizontalSpeed", params.max_vxy);
	opt("Joystick.MaxVerticalSpeed", params.max_vz);
	opt("Joystick.MaxYawRate", params.max_yaw_rate);
	opt("Joystick.CommandTimeoutMs", params.command_timeout_ms);
	opt("Test.Script", params.test_script);
	int domain = static_cast<int>(params.dds.domain_id);
	opt("DDS.Domain", domain);
	params.dds.domain_id = static_cast<std::uint32_t>(domain);
	opt("DDS.RGBTopic", params.dds.rgb_topic);
	opt("DDS.HistoryDepth", params.dds.history_depth);
	opt("DDS.SharedMemoryOnly", params.dds.shared_memory_only);
	opt("DDS.DataSharing", params.dds.data_sharing);
	auto& g = params.gains;
	opt("Flight.HoverOmega", g.hover_omega);
	opt("Flight.MaxOmega", g.max_omega);
	opt("Flight.KzP", g.k_z_p);
	opt("Flight.KzD", g.k_z_d);
	opt("Flight.KzI", g.k_z_i);
	opt("Flight.KV", g.k_v);
	opt("Flight.MaxTilt", g.max_tilt);
	opt("Flight.KRollP", g.k_roll_p);
	opt("Flight.KPitchP", g.k_pitch_p);
	opt("Flight.KRate", g.k_rate);
	opt("Flight.KYaw", g.k_yaw);
	opt("Flight.Backend", params.flight_backend);
	opt("ArduPilot.Port", params.ap_port);
	opt("ArduPilot.MaxOmega", params.ap_max_omega);
	opt("ArduPilot.TimeoutMs", params.ap_timeout_ms);
	opt("ArduPilot.MotorTimeConstant", params.motor_tau_s);
	opt("Pilot.Host", params.pilot.host);
	opt("Pilot.Port", params.pilot.port);
	opt("Pilot.TakeoffAltitude", params.pilot.takeoff_alt_m);
	opt("Pilot.LandAltitude", params.pilot.land_alt_m);

	// WEBOTS_CONTROLLER_URL names the robot this extern controller drives. The Husky's bridge names its own,
	// so two extern controllers share one Webots without racing for the same robot.
	if (std::getenv("WEBOTS_CONTROLLER_URL") == nullptr)
		setenv("WEBOTS_CONTROLLER_URL", params.webots_url.c_str(), 1);
	std::print("[drone_bridge] connecting to Webots robot at {} (waits here until that robot exists)\n",
	           std::getenv("WEBOTS_CONTROLLER_URL"));
	sim_thread_ = std::thread(&SpecificWorker::sim_loop, this);
	if (params.flight_backend == "ardupilot")
		pilot_.start(params.pilot, [this] {
			drone::PilotInput in;
			std::lock_guard lk(mtx_);
			in.selected = shared_.selected;
			if (std::chrono::steady_clock::now() - shared_.cmd_stamp < std::chrono::milliseconds(params.command_timeout_ms))
				in.cmd = shared_.cmd;                 // quiet joystick -> zero velocity -> the autopilot holds
			return in;
		});
	setPeriod("Compute", 2000);
}

void SpecificWorker::sim_loop()
{
	webots::Supervisor sup;              // blocks until Webots hands us the robot
	connected_ = true;
	const int dt_ms = static_cast<int>(sup.getBasicTimeStep());
	const double dt = dt_ms / 1000.0;

	auto* camera  = sup.getCamera(params.camera_name);
	auto* imu     = sup.getInertialUnit("inertial unit");
	auto* gyro    = sup.getGyro("gyro");
	auto* pitch_m = sup.getMotor("camera pitch");
	std::array<webots::Motor*, 4> rotors = {
		sup.getMotor("front left propeller"), sup.getMotor("front right propeller"),
		sup.getMotor("rear left propeller"),  sup.getMotor("rear right propeller")};
	if (camera == nullptr or imu == nullptr or gyro == nullptr or std::ranges::any_of(rotors, [](auto* m) { return m == nullptr; }))
	{
		std::print(stderr, "[drone_bridge] the robot behind {} is not the inspection drone (missing devices)\n",
		           params.webots_url);
		finished_ = true;
		QMetaObject::invokeMethod(QCoreApplication::instance(), "quit", Qt::QueuedConnection);
		return;
	}
	imu->enable(dt_ms);
	gyro->enable(dt_ms);
	auto* accel = sup.getAccelerometer("accelerometer");
	if (accel) accel->enable(dt_ms);

	// ArduCopter SITL: it flies the drone; this loop is only its physics. Motor order is ArduPilot's
	// FRAME_TYPE 18 (X_REV): our propellers spin opposite to the standard X — FL and RR anticlockwise seen from
	// above, FR and RL clockwise — which is exactly X_REV with 1 = rear right, 2 = front right, 3 = rear left,
	// 4 = front left. Rotor array here: 0 FL, 1 FR, 2 RL, 3 RR; spin signs {+, -, -, +}.
	const bool use_ap = params.flight_backend == "ardupilot";
	drone::ArduPilotLink ap;
	bool ap_live = false;
	std::array<double, 4> ap_omega{};
	constexpr std::array<int, 4> kServoToRotor = {3, 1, 2, 0};
	constexpr std::array<double, 4> kRotorSign = {+1.0, -1.0, -1.0, +1.0};
	bool ext_prop = false;
	double kT = 0.0, kQ = 0.0;
	std::array<std::array<double, 3>, 4> rotor_pos{};
	std::array<double, 4> rotor_w{};                  // actual (lagged) rotor speeds, signed
	if (use_ap)
	{
		if (accel == nullptr)
		{
			std::print(stderr, "[drone_bridge] Flight.Backend = ardupilot needs an Accelerometer named 'accelerometer'\n");
			finished_ = true;
			QMetaObject::invokeMethod(QCoreApplication::instance(), "quit", Qt::QueuedConnection);
			return;
		}
		if (not ap.open(params.ap_port))
		{
			std::print(stderr, "[drone_bridge] cannot bind UDP {} for ArduPilot SITL\n", params.ap_port);
			finished_ = true;
			QMetaObject::invokeMethod(QCoreApplication::instance(), "quit", Qt::QueuedConnection);
			return;
		}
		std::print("[drone_bridge] flight backend: ArduPilot SITL (JSON) on UDP {}, max omega {} rad/s\n",
		           params.ap_port, params.ap_max_omega);
		// Rotor forces. Webots' Propeller builds thrust over SECONDS here (measured: a 0.23 kg drone in free air
		// still had no thrust 0.5 s after a step to 1.3x hover, and only ~1 g after ~3.5 s), and no autopilot can
		// fly through that. With the proto's externalPropulsion = TRUE the propellers only spin and this loop
		// applies each rotor's thrust and reaction torque itself: speed through a first-order lag (time constant
		// ArduPilot.MotorTimeConstant, a real small motor is ~20-50 ms), thrust kT·w² at the rotor's centre of
		// thrust, yaw reaction -kQ·|w|·w. kT, kQ and the rotor positions are the proto's own, at its scale.
		webots::Node* me = sup.getSelf();
		if (auto* f = me->getField("externalPropulsion"); f != nullptr and f->getSFBool())
		{
			const double sc = me->getField("scale") ? me->getField("scale")->getSFFloat() : 1.0;
			ext_prop = true;
			kT = 0.00026 * sc * sc * sc;
			kQ = 0.0000052 * sc * sc * sc * sc;
			rotor_pos = {{{ 0.0548537 * sc,  0.151294 * sc, -0.00280468 * sc},     // FL
			              { 0.0548537 * sc, -0.151294 * sc, -0.00280468 * sc},     // FR
			              {-0.177179 * sc,   0.127453 * sc, -0.0320282 * sc},      // RL
			              {-0.177179 * sc,  -0.127453 * sc, -0.0320282 * sc}}};    // RR
			std::print("[drone_bridge]   rotor model: externalPropulsion, tau {} s, kT {:.3g}, kQ {:.3g}\n",
			           params.motor_tau_s, kT, kQ);
		}
		else
			std::print(stderr, "[drone_bridge]   WARNING: externalPropulsion = FALSE: thrust comes from Webots' propellers, "
			                   "which respond in seconds — set it TRUE on the InspectionDrone for ArduPilot\n");
	}
	camera->enable(params.camera_period_ms);
	for (auto* m : rotors) { m->setPosition(INFINITY); m->setVelocity(0.0); }
	if (pitch_m) pitch_m->setPosition(params.camera_pitch);
	webots::Keyboard* kb = sup.getKeyboard();
	kb->enable(dt_ms);

	webots::Node* self    = sup.getSelf();
	webots::Node* carrier = sup.getFromDef(params.carrier_def);
	webots::Node* cam_node = sup.getFromDevice(camera);
	if (carrier == nullptr)
		std::print(stderr, "[drone_bridge] no carrier DEF '{}' in the world: the pose is reported in the WORLD frame\n",
		           params.carrier_def);

	params.dds.width    = camera->getWidth();
	params.dds.height   = camera->getHeight();
	params.dds.hfov_rad = camera->getFov();
	camera_pub_.init(params.dds);

	drone::FlightController fc(params.gains);
	std::vector<std::uint8_t> rgb(static_cast<std::size_t>(params.dds.width) * params.dds.height * 3);
	double next_cam_s = 0.0, next_log_s = 0.0;
	const double t0 = sup.getTime();
	const std::array<double, 3> start_pos = {self->getPosition()[0], self->getPosition()[1], self->getPosition()[2]};
	std::print("[drone_bridge] connected: basic step {} ms, camera {}x{} @ {} ms, carrier '{}'{}\n", dt_ms,
	           params.dds.width, params.dds.height, params.camera_period_ms, params.carrier_def,
	           params.test_script.empty() ? "" : "  — TEST SCRIPT '" + params.test_script + "'");

	while (not stop_ and sup.step(dt_ms) != -1)
	{
		const double t = sup.getTime();
		const double* rpy = imu->getRollPitchYaw();
		const double* w   = gyro->getValues();
		const double* pos = self->getPosition();
		const double* vel = self->getVelocity();
		drone::FlightState st{rpy[0], rpy[1], rpy[2], w[0], w[1], w[2], pos[2], vel[0], vel[1], vel[2]};

		// ── command ──
		drone::FlightCommand cmd;
		if (params.test_script == "hover")
		{
			// scripted: idle (selected), climb, hold, forward, hold, yaw, left, hold, descend
			const double s = t - t0 - 3.0;
			if (s > 0 and s < 3)        cmd.vz = 0.6;
			else if (s > 7 and s < 10)  cmd.vx = 1.0;
			else if (s > 13 and s < 16) cmd.yaw_rate = 0.8;
			else if (s > 16 and s < 19) cmd.vy = 1.0;
			else if (s > 22)            cmd.vz = -0.5;
			if (s > 34) break;
		}
		else
		{
			std::lock_guard lk(mtx_);
			if (std::chrono::steady_clock::now() - shared_.cmd_stamp < std::chrono::milliseconds(params.command_timeout_ms))
				cmd = shared_.cmd;
		}

		// ── camera gimbal: joystick request, or F/D on the Webots keyboard ──
		double pitch_req = -1.0;
		{
			std::lock_guard lk(mtx_);
			std::swap(pitch_req, shared_.pitch_request);
		}
		for (int k = kb->getKey(); k != -1; k = kb->getKey())
		{
			if (k == 'F') pitch_req = 0.0;
			if (k == 'D') pitch_req = 1.5707963;
		}
		if (pitch_req >= 0.0 and pitch_m) pitch_m->setPosition(pitch_req);

		// ── fly ── (selected = the joystick's control target is this drone: idle the rotors while parked)
		bool selected = params.test_script == "hover";
		{
			std::lock_guard lk(mtx_);
			selected = selected or shared_.selected;
		}
		std::array<double, 4> omega{};
		if (not use_ap)
		{
			omega = fc.step(cmd, st, dt, selected);
			armed_ = fc.armed();
		}
		else
		{
			// 1) reply to the servo frame that produced the step just taken (state AFTER it)
			drone::PhysicsState ps;
			ps.time_s = t;
			const double* gv = gyro->getValues();
			const double* av = accel->getValues();
			const double* R  = self->getOrientation();
			ps.gyro_flu  = {gv[0], gv[1], gv[2]};
			ps.accel_flu = {av[0], av[1], av[2]};
			ps.pos_enu   = {pos[0] - start_pos[0], pos[1] - start_pos[1], pos[2] - start_pos[2]};
			ps.vel_enu   = {vel[0], vel[1], vel[2]};
			for (int i = 0; i < 9; ++i) ps.R_enu_flu[i] = R[i];
			if (ap_live) ap.send(ps);
			// 2) lockstep: wait for the autopilot's next output before stepping the world again
			if (auto f = ap.receive(ap_live ? params.ap_timeout_ms : 0); f.has_value())
			{
				if (not ap_live) std::print("[drone_bridge] ArduPilot SITL connected (frame rate {} Hz)\n", f->frame_rate);
				ap_live = true;
				for (int k = 0; k < 4; ++k)
				{
					const double u = std::clamp((f->pwm[k] - 1000.0) / 1000.0, 0.0, 1.0);
					const int r = kServoToRotor[k];
					ap_omega[r] = kRotorSign[r] * params.ap_max_omega * std::sqrt(u);
				}
			}
			else if (ap_live)
			{
				ap_live = false;
				ap_omega = {};
				std::print(stderr, "[drone_bridge] ArduPilot SITL silent for {} ms: rotors off, waiting for it\n",
				           params.ap_timeout_ms);
			}
			if (ext_prop)
			{
				const double a = 1.0 - std::exp(-dt / std::max(params.motor_tau_s, 1e-4));
				double tz = 0.0;
				for (int r = 0; r < 4; ++r)
				{
					rotor_w[r] += a * (ap_omega[r] - rotor_w[r]);
					const double thrust[3] = {0.0, 0.0, kT * rotor_w[r] * rotor_w[r]};
					self->addForceWithOffset(thrust, rotor_pos[r].data(), true);
					tz -= kQ * std::abs(rotor_w[r]) * rotor_w[r];
				}
				const double torque[3] = {0.0, 0.0, tz};
				self->addTorque(torque, true);
				omega = rotor_w;                          // the propellers spin for show; they make no force
			}
			else
				omega = ap_omega;
			armed_ = std::ranges::any_of(ap_omega, [](double w) { return std::abs(w) > 1.0; });
			// DRONE_AP_DEBUG=1: 20 Hz trace of rotor speeds, attitude and altitude while armed
			if (static const bool dbg = std::getenv("DRONE_AP_DEBUG") != nullptr; dbg and armed_ and t >= next_log_s)
			{
				next_log_s = t + 0.05;
				const double* e = imu->getRollPitchYaw();
				std::print("[apdbg] t={:7.3f} w(FL,FR,RL,RR)=({:+6.1f},{:+6.1f},{:+6.1f},{:+6.1f}) rpy=({:+6.3f},{:+6.3f},{:+6.3f}) "
				           "gyro=({:+5.2f},{:+5.2f},{:+5.2f}) z={:5.3f}\n", t, omega[0], omega[1], omega[2], omega[3],
				           e[0], e[1], e[2], gv[0], gv[1], gv[2], pos[2] - start_pos[2]);
			}
		}
		for (int i = 0; i < 4; ++i) rotors[i]->setVelocity(omega[i]);
		altitude_ = pos[2];

		// ── poses (robot convention: +Y forward / optical axis, +X right, +Z up) ──
		const Mat3 Rd = mul(from_webots(self->getOrientation()), kFluFromRobot);   // drone (robot conv.) in world
		Mat3 Rh = {1, 0, 0, 0, 1, 0, 0, 0, 1};
		const double zero[3] = {0, 0, 0};
		const double* ph = zero;
		if (carrier) { Rh = from_webots(carrier->getOrientation()); ph = carrier->getPosition(); }
		const Mat3 RhT = transpose(Rh);
		const Vec3 t_dc = mul(RhT, sub(pos, ph));
		const Vec3 e_dc = euler_xyz(mul(RhT, Rd));
		const Vec3 v_c  = mul(RhT, Vec3{vel[0], vel[1], vel[2]});
		const Mat3 Rc = mul(from_webots(cam_node->getOrientation()), kFluFromRobot);
		const Mat3 RdT = transpose(Rd);
		const Vec3 t_cd = mul(RdT, sub(cam_node->getPosition(), pos));
		const Mat3 R_cd = mul(RdT, Rc);
		{
			const auto now_ms = static_cast<long>(wall_ms());
			const auto sim_ms = static_cast<long>(t * 1000.0);
			std::lock_guard lk(mtx_);
			auto& e = shared_.drone_in_carrier;
			e.source = "webots-supervisor";
			e.x = t_dc[0]; e.y = t_dc[1]; e.z = t_dc[2];
			e.rx = e_dc[0]; e.ry = e_dc[1]; e.rz = e_dc[2];
			e.vx = v_c[0]; e.vy = v_c[1]; e.vz = v_c[2];
			e.poseCov = {};
			e.poseCov.m00 = e.poseCov.m11 = e.poseCov.m22 = 1e-6f;   // ground truth: tiny, not zero
			e.poseCov.m33 = e.poseCov.m44 = e.poseCov.m55 = 1e-6f;
			e.velCov = {}; e.velCov.m00 = -1.f;                      // unknown
			e.accCov = {}; e.accCov.m00 = -1.f;
			e.confidence = 100;
			e.timestamp = now_ms; e.simTimestamp = sim_ms; e.simulated = true;
			auto& m = shared_.camera_in_drone;
			m.source = "webots-supervisor";
			m.m00 = R_cd[0]; m.m01 = R_cd[1]; m.m02 = R_cd[2]; m.m03 = t_cd[0];
			m.m10 = R_cd[3]; m.m11 = R_cd[4]; m.m12 = R_cd[5]; m.m13 = t_cd[1];
			m.m20 = R_cd[6]; m.m21 = R_cd[7]; m.m22 = R_cd[8]; m.m23 = t_cd[2];
			m.m30 = 0; m.m31 = 0; m.m32 = 0; m.m33 = 1;
			m.timestamp = now_ms; m.simTimestamp = sim_ms; m.simulated = true;
			shared_.pose_valid = true;
		}

		// ── camera -> DDS ──
		if (t + 1e-9 >= next_cam_s)
		{
			next_cam_s = t + params.camera_period_ms / 1000.0;
			if (const unsigned char* bgra = camera->getImage(); bgra != nullptr)
			{
				const std::size_t n = static_cast<std::size_t>(params.dds.width) * params.dds.height;
				for (std::size_t i = 0; i < n; ++i)
				{
					rgb[3 * i + 0] = bgra[4 * i + 2];
					rgb[3 * i + 1] = bgra[4 * i + 1];
					rgb[3 * i + 2] = bgra[4 * i + 0];
				}
				if (camera_pub_.publish_rgb(wall_ms(), params.dds.width, params.dds.height, rgb.data()))
					frames_.fetch_add(1, std::memory_order_relaxed);
			}
		}

		if (not params.test_script.empty() and t >= next_log_s)
		{
			next_log_s = t + 0.25;
			std::print("[test] t={:5.2f} armed={} z={:6.3f} zref={:6.3f} v=({:+5.2f},{:+5.2f},{:+5.2f}) "
			           "rpy=({:+5.3f},{:+5.3f},{:+6.3f}) w0={:6.1f} rel=({:+5.2f},{:+5.2f},{:+5.2f})\n",
			           t - t0, fc.armed(), pos[2], fc.altitude_setpoint(), vel[0], vel[1], vel[2], rpy[0], rpy[1],
			           rpy[2], omega[0], t_dc[0], t_dc[1], t_dc[2]);
		}
	}
	for (auto* m : rotors) m->setVelocity(0.0);
	finished_ = true;
	if (not stop_)   // Webots went away (or the test script ended): nothing left to bridge
	{
		std::print("[drone_bridge] simulation ended — exiting\n");
		QMetaObject::invokeMethod(QCoreApplication::instance(), "quit", Qt::QueuedConnection);
	}
}

void SpecificWorker::compute()
{
	if (not connected_)
	{
		std::print("[drone_bridge] waiting for Webots robot {}\n", params.webots_url);
		return;
	}
	static std::uint64_t last_frames = 0;
	const auto f = frames_.load();
	std::string ap;
	if (params.flight_backend == "ardupilot")
	{
		const auto ps = pilot_.status();
		static constexpr const char* kLanded[] = {"?", "on ground", "in air", "taking off", "landing"};
		ap = not ps.connected ? " | ArduPilot: not connected"
		     : std::format(" | ArduPilot: {} mode {} {} rel.alt {:.2f} m", ps.armed ? "ARMED" : "disarmed", ps.mode,
		                   kLanded[std::clamp(ps.landed, 0, 4)], ps.rel_alt_m);
	}
	std::string rel;
	{
		std::lock_guard lk(mtx_);
		if (shared_.pose_valid)
			rel = std::format(" | from Husky (x right, y fwd, z up) ({:+.2f}, {:+.2f}, {:+.2f}) m", shared_.drone_in_carrier.x,
			                  shared_.drone_in_carrier.y, shared_.drone_in_carrier.z);
	}
	std::print("[drone_bridge] {} | altitude {:.2f} m{} | camera frames +{} | rotors {}{}\n",
	           finished_ ? "finished" : "running", altitude_.load(), rel, f - last_frames, armed_ ? "ON" : "off", ap);
	last_frames = f;
}

void SpecificWorker::emergency() {}
void SpecificWorker::restore() {}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

RoboCompFullPoseEstimation::FullPoseEuler SpecificWorker::FullPoseEstimation_getFullPoseEuler()
{
	std::lock_guard lk(mtx_);
	if (not shared_.pose_valid or finished_)
		throw RoboCompFullPoseEstimation::HardwareFailedException("drone pose not available");
	return shared_.drone_in_carrier;
}

RoboCompFullPoseEstimation::FullPoseMatrix SpecificWorker::FullPoseEstimation_getFullPoseMatrix()
{
	std::lock_guard lk(mtx_);
	if (not shared_.pose_valid or finished_)
		throw RoboCompFullPoseEstimation::HardwareFailedException("camera pose not available");
	return shared_.camera_in_drone;
}

void SpecificWorker::FullPoseEstimation_setInitialPose(float, float, float, float, float, float) {}

std::string SpecificWorker::MediaPlaneDDS_getMediaDescriptor()
{
	if (not connected_ or finished_)
		return {};
	return camera_pub_.descriptor_json();
}

void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
	if (data.id != params.mode_id)
	{
		// Addressed to the robot: the pilot took the sticks back (B). An empty id is an older publisher that
		// knows nothing about targets — it does not change the selection.
		if (not data.id.empty())
		{
			std::lock_guard lk(mtx_);
			shared_.selected = false;
		}
		return;
	}
	drone::FlightCommand c;
	double pitch = -1.0;
	for (const auto& a : data.axes)
	{
		if (a.name == params.axis_vertical)     c.vz = a.value * params.max_vz;
		else if (a.name == params.axis_yaw)     c.yaw_rate = a.value * params.max_yaw_rate;
		else if (a.name == params.axis_forward) c.vx = a.value * params.max_vxy;
		else if (a.name == params.axis_side)    c.vy = a.value * params.max_vxy;
	}
	for (const auto& b : data.buttons)
	{
		if (b.step == 0) continue;
		if (b.name == params.button_cam_front) pitch = 0.0;
		else if (b.name == params.button_cam_down) pitch = 1.5707963;
	}
	std::lock_guard lk(mtx_);
	shared_.cmd = c;
	shared_.cmd_stamp = std::chrono::steady_clock::now();
	shared_.selected = true;   // stays selected while the controller is silent (it goes quiet when idle)
	if (pitch >= 0.0) shared_.pitch_request = pitch;
}
