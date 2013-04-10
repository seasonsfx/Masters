#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "glheaders.h"
#include <unordered_map>
#include <memory>
#include <functional>
#include <QGLWidget>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include "gui/camera.h"
#include "gui/cloudgldata.h"
#include "model/cloudlist.h"
#include "model/layerlist.h"
#include "gui/gldata.h"
#include "gui/export.h"

class DLLSPEC GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QGLFormat & fmt, CloudList * cl,
             LayerList * ll, QWidget *parent = 0);
    ~GLWidget();

    void setGLD(GLData *gld);
    QSize minimumSizeHint() const;
    QSize sizeHint() const;

    void resetRotationMatrix();
    GLData *getGLData();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

protected:
   // Event handlers
   // Do I want to be aware of plugin manager?
   // Perhaps the plugin manager can sit in the parent scope
   // Keyboard events get filtered and passed to the camera
   // Should the cammera be in the parent scope?
   // What happens when plugins want to draw things?
   // Should I factor out the gl context?
   // If the gl context is in the parent scope then plugins can be passed
   // The window size really needs to be known to draw some stuff
   // Perhaps the window size can be passed in
   // Should the plugin be aware of the qgl buffers?
   // Plugins need to be called on paint events
   // Just keep things in the gl widget for now
   // Should there be 2d and 3d plugins?

   void mouseDoubleClickEvent(QMouseEvent * event);
   void mouseMoveEvent(QMouseEvent * event);
   void mousePressEvent(QMouseEvent * event);
   void mouseReleaseEvent(QMouseEvent * event);
   void wheelEvent(QWheelEvent * event);
   void keyPressEvent(QKeyEvent * event);
   bool eventFilter(QObject *object, QEvent *event);

 signals:
   void pluginPaint(const Eigen::Affine3f& proj, const Eigen::Affine3f& mv);

 private slots:
  void contextMenu(const QPoint &pos);

 public:
   Camera camera_;

 private:
    CloudList * cl_;
    LayerList * ll_;
    GLData * gld_;

    QGLShaderProgram program_;

    int uni_sampler_;
    int uni_projection_;
    int uni_modelview_;
    int uni_select_color_;

    float translate_unit_;
    QVector2D mouse_drag_start_;
    float point_render_size_;

    GLuint texture_id_;
    GLuint vao_;

    Eigen::Vector2d last_mouse_pos_;
};

#endif

