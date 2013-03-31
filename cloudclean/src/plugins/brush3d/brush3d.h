#ifndef BRUSH_3D_H
#define BRUSH_3D_H

#include "pluginsystem/iplugin.h"

#include <Eigen/Dense>

#include "glheaders.h"

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;
class QAction;
class QGLShaderProgram;
class QGLBuffer;
class Core;
class CloudList;
class LayerList;
class FlatView;
class ActionManager;
class GLWidget;

class Brush3D : public IPlugin {
    Q_OBJECT
    Q_INTERFACES(IPlugin)
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();

    void initializeGL();

    bool eventFilter(QObject *object, QEvent *event);

 signals:
    void enabling();

 public slots:
    void enable();
    void disable();
    void paint(Eigen::Affine3f, Eigen::Affine3f);

 private:
    void select(QMouseEvent * event);
    bool mouseClickEvent(QMouseEvent * event);
    bool mouseMoveEvent(QMouseEvent * event);
    bool mousePressEvent(QMouseEvent * event);
    bool mouseReleaseEvent(QMouseEvent * event);

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    bool initialized_gl;
    ActionManager * am_;
    QAction * enable_;

    Eigen::Vector2d last_mouse_pos_;
    Eigen::Vector2d mouse_down_pos_;

    Eigen::Vector3f p1;
    Eigen::Vector3f p2;
    QGLShaderProgram * program_;
    QGLBuffer * line_;

    bool is_enabled_;

};

#endif  // BRUSH_3D_H
