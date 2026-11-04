#ifndef RC_COMMON_GL_WORLD3D_VIEWER_H
#define RC_COMMON_GL_WORLD3D_VIEWER_H

/*
 *  Reusable 3D "simple world" viewer (active_inference/common): coloured primitives in a z-up world.
 *
 *  The 3D semantic map an agent shows of the world it believes in — boxes (a robot), cylinders and cones (a
 *  trunk, a canopy), flat strips on the ground (a row, a plot, the field) — built by the caller as a list of
 *  coloured triangles in the WORLD frame (x east, y north, z up, metres). Flat shading, one-sided lighting with
 *  an ambient floor, orbit/pan/zoom camera that starts looking at the scene from the south-west. Pure renderer:
 *  it knows nothing about DSR; geometry comes in through set_scene(). Non-Q_OBJECT (owner-driven) → no MOC.
 *  Built on the same pattern as gl_mesh_viewer.h.
 *
 *  Helpers (World3DScene) append primitives so the caller writes "a trunk here" rather than vertex lists.
 */

#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPainter>
#include <QVector3D>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace rc::viewers
{

// A list of coloured triangles in the world frame (z up), with helpers for the primitives a world is made of.
struct World3DScene
{
    struct Vertex { float x, y, z, nx, ny, nz, r, g, b; };
    std::vector<Vertex> verts;
    std::string caption;   // drawn top-left

    void tri(const QVector3D &a, const QVector3D &b, const QVector3D &c, const QVector3D &rgb)
    {
        QVector3D n = QVector3D::crossProduct(b - a, c - a);
        if (n.lengthSquared() > 0.0f) n.normalize();
        for (const auto &v : {a, b, c})
            verts.push_back({v.x(), v.y(), v.z(), n.x(), n.y(), n.z(), rgb.x(), rgb.y(), rgb.z()});
    }
    void quad(const QVector3D &a, const QVector3D &b, const QVector3D &c, const QVector3D &d, const QVector3D &rgb)
    {
        tri(a, b, c, rgb);
        tri(a, c, d, rgb);
    }

    // An oriented box standing on z0: centre (x, y), size (length along yaw, width across, height).
    void box(float x, float y, float z0, float yaw, float length, float width, float height, const QVector3D &rgb)
    {
        const float c = std::cos(yaw), s = std::sin(yaw), hl = 0.5f * length, hw = 0.5f * width;
        const auto P = [&](float u, float v, float z) { return QVector3D(x + u * c - v * s, y + u * s + v * c, z); };
        const float z1 = z0 + height;
        const QVector3D b0 = P(-hl, -hw, z0), b1 = P(hl, -hw, z0), b2 = P(hl, hw, z0), b3 = P(-hl, hw, z0);
        const QVector3D t0 = P(-hl, -hw, z1), t1 = P(hl, -hw, z1), t2 = P(hl, hw, z1), t3 = P(-hl, hw, z1);
        quad(t0, t1, t2, t3, rgb);  quad(b3, b2, b1, b0, rgb);
        quad(b0, b1, t1, t0, rgb);  quad(b1, b2, t2, t1, rgb);
        quad(b2, b3, t3, t2, rgb);  quad(b3, b0, t0, t3, rgb);
    }

    // A vertical frustum (cylinder when r0 == r1, cone when r1 == 0) from z0 to z1 around (x, y).
    void frustum(float x, float y, float z0, float z1, float r0, float r1, const QVector3D &rgb, int sides = 16)
    {
        for (int k = 0; k < sides; ++k)
        {
            const float a0 = 2.0f * static_cast<float>(M_PI) * k / sides, a1 = 2.0f * static_cast<float>(M_PI) * (k + 1) / sides;
            const QVector3D p0(x + r0 * std::cos(a0), y + r0 * std::sin(a0), z0), p1(x + r0 * std::cos(a1), y + r0 * std::sin(a1), z0);
            const QVector3D q0(x + r1 * std::cos(a0), y + r1 * std::sin(a0), z1), q1(x + r1 * std::cos(a1), y + r1 * std::sin(a1), z1);
            if (r1 > 0.0f) quad(p0, p1, q1, q0, rgb);
            else tri(p0, p1, QVector3D(x, y, z1), rgb);
            tri(QVector3D(x, y, z0), p1, p0, rgb);                       // bottom cap
            if (r1 > 0.0f) tri(QVector3D(x, y, z1), q0, q1, rgb);         // top cap
        }
    }

    // A flat ring on the ground (a "here" mark): inner radius ri, outer ro, at height z.
    void ring(float x, float y, float z, float ri, float ro, const QVector3D &rgb, int sides = 24)
    {
        for (int k = 0; k < sides; ++k)
        {
            const float a0 = 2.0f * static_cast<float>(M_PI) * k / sides, a1 = 2.0f * static_cast<float>(M_PI) * (k + 1) / sides;
            quad(QVector3D(x + ri * std::cos(a0), y + ri * std::sin(a0), z), QVector3D(x + ro * std::cos(a0), y + ro * std::sin(a0), z),
                 QVector3D(x + ro * std::cos(a1), y + ro * std::sin(a1), z), QVector3D(x + ri * std::cos(a1), y + ri * std::sin(a1), z), rgb);
        }
    }

    // The outline of an oriented rectangle lying on the ground (a row, a plot, the field), `w` thick, at height z.
    void outline(float x, float y, float yaw, float length, float width, float z, float w, const QVector3D &rgb)
    {
        const float c = std::cos(yaw), s = std::sin(yaw);
        const auto P = [&](float u, float v) { return QVector3D(x + u * c - v * s, y + u * s + v * c, z); };
        const float hl = 0.5f * length, hw = 0.5f * width;
        const auto bar = [&](float u0, float v0, float u1, float v1) { quad(P(u0, v0), P(u1, v0), P(u1, v1), P(u0, v1), rgb); };
        bar(-hl, -hw, hl, -hw + w);  bar(-hl, hw - w, hl, hw);
        bar(-hl, -hw, -hl + w, hw);  bar(hl - w, -hw, hl, hw);
    }

    // A convex polygon on the ground (fan triangulation), at height z.
    void ground(const std::vector<QVector3D> &poly, const QVector3D &rgb)
    {
        for (std::size_t i = 1; i + 1 < poly.size(); ++i)
            tri(poly[0], poly[i], poly[i + 1], rgb);
    }
};

class GLWorld3DViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    explicit GLWorld3DViewer(QWidget *parent = nullptr) : QOpenGLWidget(parent)
    {
        resize(900, 700);
        setWindowTitle("world 3D");
        setFocusPolicy(Qt::StrongFocus);
    }

    ~GLWorld3DViewer() override
    {
        if (context())
        {
            makeCurrent();
            if (vbo_.isCreated()) vbo_.destroy();
            if (vao_.isCreated()) vao_.destroy();
            doneCurrent();
        }
    }

    void set_scene(World3DScene scene)
    {
        scene_ = std::move(scene);
        float minx = std::numeric_limits<float>::max(), miny = minx, maxx = std::numeric_limits<float>::lowest(), maxy = maxx;
        for (const auto &v : scene_.verts)
        {
            minx = std::min(minx, v.x); maxx = std::max(maxx, v.x);
            miny = std::min(miny, v.y); maxy = std::max(maxy, v.y);
        }
        if (not scene_.verts.empty() and not user_interacted_)
        {
            center_ = QVector3D(0.5f * (minx + maxx), 0.5f * (miny + maxy), 0.0f);
            cam_dist_ = std::max(5.0f, 0.9f * std::max(maxx - minx, maxy - miny));
        }
        upload();
        update();
    }

protected:
    void initializeGL() override
    {
        initializeOpenGLFunctions();
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.80f, 0.87f, 0.94f, 1.0f);   // sky
        program_.addShaderFromSourceCode(QOpenGLShader::Vertex, R"(
            #version 330 core
            layout(location = 0) in vec3 position;
            layout(location = 1) in vec3 normal;
            layout(location = 2) in vec3 color;
            uniform mat4 u_mvp;
            out vec3 v_normal;
            out vec3 v_color;
            void main() { gl_Position = u_mvp * vec4(position, 1.0); v_normal = normal; v_color = color; }
        )");
        program_.addShaderFromSourceCode(QOpenGLShader::Fragment, R"(
            #version 330 core
            in vec3 v_normal;
            in vec3 v_color;
            out vec4 fragColor;
            uniform vec3 u_light;
            void main()
            {
                float d = abs(dot(normalize(v_normal), u_light));
                fragColor = vec4(v_color * (0.45 + 0.55 * d), 1.0);
            }
        )");
        program_.link();
        u_mvp_ = program_.uniformLocation("u_mvp");
        u_light_ = program_.uniformLocation("u_light");
        vao_.create();
        vbo_.create();
        upload();
    }

    void resizeGL(int w, int h) override { glViewport(0, 0, w, h); }

    void paintGL() override
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (not program_.isLinked())
            return;
        const float aspect = static_cast<float>(std::max(1, width())) / static_cast<float>(std::max(1, height()));
        QMatrix4x4 proj;
        proj.perspective(45.0f, aspect, 0.05f, cam_dist_ * 20.0f + 100.0f);
        QMatrix4x4 view;
        view.translate(pan_x_, pan_y_, -cam_dist_);
        view.rotate(pitch_deg_, 1.0f, 0.0f, 0.0f);   // tilt the ground towards the camera
        view.rotate(yaw_deg_, 0.0f, 0.0f, 1.0f);     // spin around the world's z (up)
        view.translate(-center_);
        program_.bind();
        program_.setUniformValue(u_mvp_, proj * view);
        program_.setUniformValue(u_light_, QVector3D(0.35f, -0.45f, 0.82f).normalized());
        vao_.bind();
        if (not scene_.verts.empty())
        {
            vbo_.bind();
            using V = World3DScene::Vertex;
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(V), nullptr);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(V), reinterpret_cast<const void *>(offsetof(V, nx)));
            glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(V), reinterpret_cast<const void *>(offsetof(V, r)));
            for (int a = 0; a < 3; ++a) glEnableVertexAttribArray(a);
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(scene_.verts.size()));
            for (int a = 0; a < 3; ++a) glDisableVertexAttribArray(a);
            vbo_.release();
        }
        vao_.release();
        program_.release();

        QPainter painter(this);
        painter.setRenderHint(QPainter::TextAntialiasing, true);
        painter.setPen(QColor(25, 35, 45));
        painter.drawText(QRect(10, 8, width() - 20, 60), Qt::AlignLeft | Qt::AlignTop,
                         QString::fromStdString(scene_.caption) +
                         QStringLiteral("\n[drag = rotate · right/middle = pan · wheel = zoom · R = reset]"));
    }

    void mousePressEvent(QMouseEvent *e) override
    {
        last_ = e->pos();
        if (e->button() == Qt::LeftButton) { rotating_ = true; user_interacted_ = true; }
        else if (e->button() == Qt::RightButton or e->button() == Qt::MiddleButton) { panning_ = true; user_interacted_ = true; }
    }
    void mouseMoveEvent(QMouseEvent *e) override
    {
        const QPoint d = e->pos() - last_;
        last_ = e->pos();
        if (rotating_)
        {
            yaw_deg_ += d.x() * 0.4f;
            pitch_deg_ = std::clamp(pitch_deg_ + d.y() * 0.4f, -89.0f, 0.0f);
            update();
        }
        else if (panning_)
        {
            const float s = std::max(0.001f, cam_dist_ * 0.0015f);
            pan_x_ += d.x() * s;
            pan_y_ -= d.y() * s;
            update();
        }
    }
    void mouseReleaseEvent(QMouseEvent *) override { rotating_ = false; panning_ = false; }
    void wheelEvent(QWheelEvent *e) override
    {
        const float steps = e->angleDelta().y() / 120.0f;
        cam_dist_ = std::clamp(cam_dist_ * std::pow(0.85f, steps), 1.0f, 2000.0f);
        user_interacted_ = true;
        update();
    }
    void keyPressEvent(QKeyEvent *e) override
    {
        if (e->key() == Qt::Key_R)
        {
            yaw_deg_ = -30.0f; pitch_deg_ = -55.0f; pan_x_ = pan_y_ = 0.0f; user_interacted_ = false;
            set_scene(scene_);
            return;
        }
        QOpenGLWidget::keyPressEvent(e);
    }

private:
    void upload()
    {
        if (not context() or not vbo_.isCreated())
            return;
        makeCurrent();
        vbo_.bind();
        vbo_.allocate(scene_.verts.data(), static_cast<int>(scene_.verts.size() * sizeof(World3DScene::Vertex)));
        vbo_.release();
        doneCurrent();
    }

    World3DScene scene_;
    QVector3D center_{0, 0, 0};
    float cam_dist_ = 30.0f, yaw_deg_ = -30.0f, pitch_deg_ = -55.0f, pan_x_ = 0.0f, pan_y_ = 0.0f;
    bool rotating_ = false, panning_ = false, user_interacted_ = false;
    QPoint last_;
    QOpenGLShaderProgram program_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_{QOpenGLBuffer::VertexBuffer};
    int u_mvp_ = -1, u_light_ = -1;
};

}   // namespace rc::viewers

#endif   // RC_COMMON_GL_WORLD3D_VIEWER_H
