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
#include <print>
#include <vector>

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

	// WEBOTS_CONTROLLER_URL names the robot this extern controller drives. The Husky's bridge names its own,
	// so two extern controllers share one Webots without racing for the same robot.
	if (std::getenv("WEBOTS_CONTROLLER_URL") == nullptr)
		setenv("WEBOTS_CONTROLLER_URL", params.webots_url.c_str(), 1);
	std::print("[drone_bridge] connecting to Webots robot at {} (waits here until that robot exists)\n",
	           std::getenv("WEBOTS_CONTROLLER_URL"));
	sim_thread_ = std::thread(&SpecificWorker::sim_loop, this);
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
		const auto omega = fc.step(cmd, st, dt, selected);
		for (int i = 0; i < 4; ++i) rotors[i]->setVelocity(omega[i]);
		armed_ = fc.armed();
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
	std::print("[drone_bridge] {} | altitude {:.2f} m | camera frames +{} | rotors {}\n",
	           finished_ ? "finished" : "running", altitude_.load(), f - last_frames, armed_ ? "ON" : "off");
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
