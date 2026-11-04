// openfield_viewer.h — the openfield_concept window: where the robot thinks it is, what each sensor says,
// and switches to take the GPS (and the IMU's absolute yaw) out of the filter, to watch the dead-reckoning
// error grow and then collapse when the GPS comes back.
//
// Pure presentation: the worker pushes a ViewerState at ~10 Hz and listens to the toggles. Everything runs
// on the Qt main thread (compute() is driven by the agent's state-machine timers), so nothing is locked.
#pragma once

#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <Eigen/Dense>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace rc::openfield
{

struct ViewerState
{
    double t_s = 0.0;                                   // seconds since the agent started
    bool initialized = false;
    Eigen::Vector3d pose = Eigen::Vector3d::Zero();     // [x, y, theta] field frame
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
    // GPS
    bool gps_enabled = true;
    std::optional<Eigen::Vector2d> gps_fix;             // latest antenna fix (shown even when not fused)
    std::uint64_t gps_fixes = 0, gps_fused = 0;
    double gps_age_s = -1.0;                            // since the last NEW fix
    double gps_innovation_m = 0.0;                      // |fix - predicted antenna| at the last fusion
    double seconds_without_gps = 0.0;                   // since the GPS was switched off (0 while on)
    // Odometry + IMU
    bool yaw_enabled = true;
    float adv = 0.f, side = 0.f, wheel_rot = 0.f;
    double gyro_z = 0.0;
    std::optional<double> imu_yaw;
    double imu_age_ms = -1.0;
    // Ground truth (simulation only)
    std::optional<Eigen::Vector3d> gt;
    std::optional<double> err_xy, err_theta;
    std::vector<Eigen::Vector2f> polygon;
};

// The 2D view: traces, robot glyph, 2-sigma ellipse, field polygon. Follows the robot; wheel zooms.
class OpenFieldMap : public QWidget
{
    Q_OBJECT
public:
    explicit OpenFieldMap(QWidget *parent = nullptr);
    void push(const ViewerState &s);
    void clear_traces();

protected:
    void paintEvent(QPaintEvent *) override;
    void wheelEvent(QWheelEvent *e) override;

private:
    ViewerState last_;
    std::deque<Eigen::Vector2d> est_, gt_, gps_;
    std::deque<bool> est_gps_on_;                       // was the GPS fused when this estimate was made
    double scale_px_per_m_ = 25.0;
};

// Error over time: true position error (vs GT) and the filter's own 2-sigma, last 3 minutes.
class OpenFieldErrorPlot : public QWidget
{
    Q_OBJECT
public:
    explicit OpenFieldErrorPlot(QWidget *parent = nullptr);
    void push(const ViewerState &s);
    void clear();

protected:
    void paintEvent(QPaintEvent *) override;

private:
    struct Sample { double t; double err; double two_sigma; bool gps; };
    std::deque<Sample> samples_;
};

class OpenFieldViewer : public QWidget
{
    Q_OBJECT
public:
    explicit OpenFieldViewer(QWidget *parent = nullptr);
    void update_state(const ViewerState &s);
    void set_gps_checked(bool on) { gps_box_->setChecked(on); }   // as if the user clicked (emits gpsToggled)

signals:
    void gpsToggled(bool enabled);
    void yawToggled(bool enabled);

private:
    OpenFieldMap *map_ = nullptr;
    OpenFieldErrorPlot *plot_ = nullptr;
    QCheckBox *gps_box_ = nullptr, *yaw_box_ = nullptr;
    QPushButton *clear_btn_ = nullptr;
    QLabel *pose_lbl_ = nullptr, *gps_lbl_ = nullptr, *odom_lbl_ = nullptr, *imu_lbl_ = nullptr, *gt_lbl_ = nullptr;
    QLabel *banner_ = nullptr;
};

} // namespace rc::openfield
