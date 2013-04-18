#ifndef BRUSH_2D_H
#define BRUSH_2D_H

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
class FlatView;
class MainWindow;
class QUndoStack;

class Brush2D : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "brush2d.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    bool eventFilter(QObject *object, QEvent *event);

 signals:
    void enabling();

 private slots:
    void setRad(int val);

 public slots:
    void enable();
    void disable();

 private:
    void select(QMouseEvent * event);
    bool mouseMoveEvent(QMouseEvent * event);
    bool mousePressEvent(QMouseEvent * event);
    bool mouseReleaseEvent(QMouseEvent * event);
    bool mouseWheelEvent(QWheelEvent * event);

 private:
    Core * core_;
    CloudList * cl_;
    FlatView * flatview_;
    MainWindow * mw_;
    QUndoStack * us_;

    QAction * enable_;

    Eigen::Vector2d last_mouse_pos_;
    Eigen::Vector2d mouse_down_pos_;

    bool is_enabled_;
    int radius_;

    QWidget * settings_;

};

#endif  // BRUSH_2D_H
