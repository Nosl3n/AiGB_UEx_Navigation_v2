#include "openfield_viewer.h"

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace rc::openfield
{

namespace
{
constexpr std::size_t kMaxTrace = 20000;
const QColor kEst(40, 110, 230), kGt(30, 160, 60), kGps(240, 140, 20), kNoGps(200, 40, 40);

template <class T> void push_capped(std::deque<T> &d, const T &v)
{
    d.push_back(v);
    if (d.size() > kMaxTrace) d.pop_front();
}

QString fmt(double v, int prec = 3) { return QString::number(v, 'f', prec); }
} // namespace

// ── Map ─────────────────────────────────────────────────────────────────────────────────────────────
OpenFieldMap::OpenFieldMap(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(520, 520);
    setAutoFillBackground(true);
    QPalette p = palette();
    p.setColor(QPalette::Window, QColor(250, 250, 247));
    setPalette(p);
}

void OpenFieldMap::push(const ViewerState &s)
{
    last_ = s;
    if (s.initialized)
    {
        const Eigen::Vector2d e = s.pose.head<2>();
        if (est_.empty() or (est_.back() - e).norm() > 0.01)
        {
            push_capped(est_, e);
            push_capped(est_gps_on_, s.gps_enabled);
        }
    }
    if (s.gt.has_value())
    {
        const Eigen::Vector2d g = s.gt->head<2>();
        if (gt_.empty() or (gt_.back() - g).norm() > 0.01) push_capped(gt_, g);
    }
    if (s.gps_fix.has_value() and (gps_.empty() or (gps_.back() - *s.gps_fix).norm() > 1e-4))
        push_capped(gps_, *s.gps_fix);
    update();
}

void OpenFieldMap::clear_traces()
{
    est_.clear(); est_gps_on_.clear(); gt_.clear(); gps_.clear();
    update();
}

void OpenFieldMap::wheelEvent(QWheelEvent *e)
{
    scale_px_per_m_ = std::clamp(scale_px_per_m_ * (e->angleDelta().y() > 0 ? 1.2 : 1.0 / 1.2), 2.0, 400.0);
    update();
}

void OpenFieldMap::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const double s = scale_px_per_m_;
    const Eigen::Vector2d c = last_.initialized ? Eigen::Vector2d(last_.pose.head<2>())
                            : (last_.gt ? Eigen::Vector2d(last_.gt->head<2>()) : Eigen::Vector2d::Zero());
    const QPointF mid(width() / 2.0, height() / 2.0);
    const auto W = [&](const Eigen::Vector2d &w) { return QPointF(mid.x() + (w.x() - c.x()) * s, mid.y() - (w.y() - c.y()) * s); };

    // Grid: 1 m when zoomed in, 5 m otherwise; north is up.
    const double step = s > 20 ? 1.0 : 5.0;
    p.setPen(QPen(QColor(225, 225, 220), 1));
    const double hx = width() / (2 * s), hy = height() / (2 * s);
    for (double x = std::floor((c.x() - hx) / step) * step; x <= c.x() + hx; x += step)
        p.drawLine(W({x, c.y() - hy}), W({x, c.y() + hy}));
    for (double y = std::floor((c.y() - hy) / step) * step; y <= c.y() + hy; y += step)
        p.drawLine(W({c.x() - hx, y}), W({c.x() + hx, y}));

    // Field boundary.
    if (last_.polygon.size() >= 3)
    {
        QPolygonF poly;
        for (const auto &v : last_.polygon) poly << W(v.cast<double>());
        p.setPen(QPen(QColor(120, 120, 120), 2, Qt::DashLine));
        p.setBrush(Qt::NoBrush);
        p.drawPolygon(poly);
    }

    // GPS fixes (antenna), ground truth, estimate (red where the GPS was off).
    p.setPen(Qt::NoPen);
    p.setBrush(kGps);
    for (const auto &g : gps_) p.drawEllipse(W(g), 1.8, 1.8);
    if (gt_.size() > 1)
    {
        QPainterPath path(W(gt_.front()));
        for (const auto &g : gt_) path.lineTo(W(g));
        p.setPen(QPen(kGt, 2));
        p.setBrush(Qt::NoBrush);
        p.drawPath(path);
    }
    for (std::size_t i = 1; i < est_.size(); ++i)
    {
        p.setPen(QPen(est_gps_on_[i] ? kEst : kNoGps, 2));
        p.drawLine(W(est_[i - 1]), W(est_[i]));
    }

    if (last_.initialized)
    {
        // 2-sigma position ellipse.
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(last_.cov.topLeftCorner<2, 2>());
        const Eigen::Vector2d ev = es.eigenvalues().cwiseMax(0.0);
        const Eigen::Vector2d ax = es.eigenvectors().col(1);
        p.save();
        p.translate(W(last_.pose.head<2>()));
        p.rotate(-std::atan2(ax.y(), ax.x()) * 180.0 / M_PI);
        p.setPen(QPen(QColor(40, 110, 230, 160), 1.5));
        p.setBrush(QColor(40, 110, 230, 40));
        const double a = std::max(2.0 * std::sqrt(ev(1)) * s, 2.0), b = std::max(2.0 * std::sqrt(ev(0)) * s, 2.0);
        p.drawEllipse(QPointF(0, 0), a, b);
        p.restore();

        // Robot body (Husky footprint 0.69 x 0.99 m) and its heading. Robot frame: x right, y forward.
        const double th = last_.pose(2), ct = std::cos(th), st = std::sin(th);
        const auto B = [&](double bx, double by)
        { return W({last_.pose(0) + ct * bx - st * by, last_.pose(1) + st * bx + ct * by}); };
        QPolygonF body;
        body << B(-0.345, -0.495) << B(0.345, -0.495) << B(0.345, 0.495) << B(-0.345, 0.495);
        p.setPen(QPen(last_.gps_enabled ? kEst : kNoGps, 2));
        p.setBrush(QColor(255, 255, 255, 180));
        p.drawPolygon(body);
        p.drawLine(B(0, 0), B(0, 0.8));
    }

    // Legend + scale.
    p.setPen(Qt::black);
    int y = 18;
    const auto legend = [&](const QColor &col, const QString &txt)
    { p.fillRect(10, y - 9, 18, 4, col); p.drawText(34, y, txt); y += 16; };
    legend(kEst, "estimated (GPS on)");
    legend(kNoGps, "estimated without GPS");
    legend(kGt, "ground truth (Webots)");
    legend(kGps, "GPS fixes (antenna)");
    p.drawText(10, height() - 10, QString("scale: %1 m per square · wheel = zoom · north up").arg(step, 0, 'f', 0));
}

// ── Error plot ──────────────────────────────────────────────────────────────────────────────────────
OpenFieldErrorPlot::OpenFieldErrorPlot(QWidget *parent) : QWidget(parent)
{
    setMinimumHeight(170);
}

void OpenFieldErrorPlot::push(const ViewerState &s)
{
    if (not s.initialized or not s.err_xy.has_value()) return;
    const double two_sigma = 2.0 * std::sqrt(std::max(0.0, 0.5 * (s.cov(0, 0) + s.cov(1, 1))));
    samples_.push_back({s.t_s, *s.err_xy, two_sigma, s.gps_enabled});
    while (not samples_.empty() and samples_.back().t - samples_.front().t > 180.0) samples_.pop_front();
    update();
}

void OpenFieldErrorPlot::clear() { samples_.clear(); update(); }

void OpenFieldErrorPlot::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(250, 250, 247));
    const QRectF r(48, 10, width() - 58, height() - 32);
    p.setPen(QColor(180, 180, 180));
    p.drawRect(r);
    if (samples_.size() < 2) { p.drawText(r, Qt::AlignCenter, "position error against the GT (no data yet)"); return; }

    const double t1 = samples_.back().t, t0 = std::max(samples_.front().t, t1 - 180.0);
    double ymax = 0.05;
    for (const auto &q : samples_) ymax = std::max({ymax, q.err, q.two_sigma});
    ymax *= 1.1;
    const auto P = [&](double t, double v)
    { return QPointF(r.left() + (t - t0) / std::max(1e-6, t1 - t0) * r.width(), r.bottom() - v / ymax * r.height()); };

    // Shade the stretches with the GPS switched off.
    for (std::size_t i = 1; i < samples_.size(); ++i)
        if (not samples_[i].gps)
            p.fillRect(QRectF(P(samples_[i - 1].t, ymax), P(samples_[i].t, 0)), QColor(200, 40, 40, 30));

    QPainterPath err(P(samples_.front().t, samples_.front().err)), sig(P(samples_.front().t, samples_.front().two_sigma));
    for (const auto &q : samples_) { err.lineTo(P(q.t, q.err)); sig.lineTo(P(q.t, q.two_sigma)); }
    p.setPen(QPen(QColor(40, 110, 230), 1.5, Qt::DashLine)); p.drawPath(sig);
    p.setPen(QPen(QColor(20, 20, 20), 2)); p.drawPath(err);

    p.setPen(Qt::black);
    p.drawText(QRectF(0, r.top() - 4, 46, 14), Qt::AlignRight, QString::number(ymax, 'f', 2) + " m");
    p.drawText(QRectF(0, r.bottom() - 10, 46, 14), Qt::AlignRight, "0");
    p.drawText(QRectF(r.left(), r.bottom() + 4, r.width(), 16), Qt::AlignLeft,
               "— real error (m)    - - estimated 2σ    red = GPS off    (last 3 min)");
}

// ── Window ──────────────────────────────────────────────────────────────────────────────────────────
OpenFieldViewer::OpenFieldViewer(QWidget *parent) : QWidget(parent)
{
    setWindowTitle("openfield_concept — GPS + inertial odometry localization");
    map_ = new OpenFieldMap(this);
    plot_ = new OpenFieldErrorPlot(this);

    gps_box_ = new QCheckBox("GPS (position correction)", this);
    gps_box_->setChecked(true);
    yaw_box_ = new QCheckBox("IMU absolute yaw (heading correction)", this);
    yaw_box_->setChecked(true);
    clear_btn_ = new QPushButton("Clear trajectories", this);
    connect(gps_box_, &QCheckBox::toggled, this, &OpenFieldViewer::gpsToggled);
    connect(yaw_box_, &QCheckBox::toggled, this, &OpenFieldViewer::yawToggled);
    connect(clear_btn_, &QPushButton::clicked, this, [this] { map_->clear_traces(); plot_->clear(); });

    banner_ = new QLabel(this);
    banner_->setWordWrap(true);
    const auto group = [this](const QString &title, QLabel *&lbl)
    {
        auto *g = new QGroupBox(title, this);
        lbl = new QLabel(g);
        lbl->setTextFormat(Qt::RichText);
        lbl->setTextInteractionFlags(Qt::TextSelectableByMouse);
        auto *l = new QVBoxLayout(g);
        l->addWidget(lbl);
        return g;
    };
    auto *side = new QVBoxLayout;
    side->addWidget(banner_);
    side->addWidget(gps_box_);
    side->addWidget(yaw_box_);
    side->addWidget(clear_btn_);
    side->addWidget(group("Estimated pose (EKF)", pose_lbl_));
    side->addWidget(group("GPS", gps_lbl_));
    side->addWidget(group("Wheel odometry", odom_lbl_));
    side->addWidget(group("IMU", imu_lbl_));
    side->addWidget(group("Ground truth (Webots)", gt_lbl_));
    side->addStretch(1);

    auto *left = new QVBoxLayout;
    left->addWidget(map_, 1);
    left->addWidget(plot_);
    auto *top = new QHBoxLayout(this);
    top->addLayout(left, 1);
    auto *sidew = new QWidget(this);
    sidew->setLayout(side);
    sidew->setFixedWidth(330);
    top->addWidget(sidew);
    resize(1180, 820);
}

void OpenFieldViewer::update_state(const ViewerState &s)
{
    map_->push(s);
    plot_->push(s);

    if (not s.initialized)
        banner_->setText("<b>Waiting</b> for a GPS fix and the IMU yaw to initialise.");
    else if (not s.gps_enabled)
        banner_->setText(QString("<span style='color:#c82828'><b>GPS OFF</b> — odometry only%1 · %2 s</span>")
                             .arg(s.yaw_enabled ? " + IMU" : " + gyroscope").arg(s.seconds_without_gps, 0, 'f', 1));
    else
        banner_->setText("<span style='color:#2868d0'><b>GPS + odometry + IMU</b></span>");

    const double sx = std::sqrt(s.cov(0, 0)), sy = std::sqrt(s.cov(1, 1)), st = std::sqrt(s.cov(2, 2));
    pose_lbl_->setText(s.initialized
        ? QString("x = %1 m<br>y = %2 m<br>θ = %3 rad (%4°)<br>σ<sub>x</sub> = %5 m · σ<sub>y</sub> = %6 m<br>σ<sub>θ</sub> = %7°")
              .arg(fmt(s.pose(0))).arg(fmt(s.pose(1))).arg(fmt(s.pose(2))).arg(fmt(s.pose(2) * 180 / M_PI, 1))
              .arg(fmt(sx)).arg(fmt(sy)).arg(fmt(st * 180 / M_PI, 2))
        : QString("—"));
    gps_lbl_->setText(QString("state: <b>%1</b><br>last fix: %2<br>fixes received: %3 · fused: %4<br>"
                              "age: %5<br>innovation: %6 m")
        .arg(s.gps_enabled ? "fusing" : "<span style='color:#c82828'>ignored</span>")
        .arg(s.gps_fix ? QString("(%1, %2) m").arg(fmt(s.gps_fix->x())).arg(fmt(s.gps_fix->y())) : QString("—"))
        .arg(s.gps_fixes).arg(s.gps_fused)
        .arg(s.gps_age_s >= 0 ? fmt(s.gps_age_s, 2) + " s" : QString("—"))
        .arg(fmt(s.gps_innovation_m)));
    odom_lbl_->setText(QString("forward = %1 m/s<br>lateral = %2 m/s<br>turn (wheels) = %3 rad/s")
        .arg(fmt(s.adv)).arg(fmt(s.side)).arg(fmt(s.wheel_rot)));
    imu_lbl_->setText(QString("gyroscope z = %1 rad/s<br>yaw = %2<br>absolute yaw: <b>%3</b><br>age: %4 ms")
        .arg(fmt(s.gyro_z))
        .arg(s.imu_yaw ? QString("%1 rad (%2°)").arg(fmt(*s.imu_yaw)).arg(fmt(*s.imu_yaw * 180 / M_PI, 1)) : QString("—"))
        .arg(s.yaw_enabled ? "fusing" : "<span style='color:#c82828'>ignored (gyroscope only)</span>")
        .arg(s.imu_age_ms >= 0 ? fmt(s.imu_age_ms, 0) : QString("—")));
    gt_lbl_->setText(s.gt
        ? QString("x = %1 · y = %2 m<br>θ = %3°<br><b>position error = %4 m</b><br><b>heading error = %5°</b>")
              .arg(fmt(s.gt->x())).arg(fmt(s.gt->y())).arg(fmt((*s.gt)(2) * 180 / M_PI, 1))
              .arg(s.err_xy ? fmt(*s.err_xy) : QString("—"))
              .arg(s.err_theta ? fmt(*s.err_theta * 180 / M_PI, 2) : QString("—"))
        : QString("not available (simulation only)"));
}

} // namespace rc::openfield
