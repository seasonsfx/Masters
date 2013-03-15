#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "glheaders.h"
#include <unordered_map>
#include <memory>
#include <functional>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include "gui/camera.h"
#include "gui/cloudgldata.h"
#include "model/cloudlist.h"
#include "model/layerlist.h"
#include "gui/gldata.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QGLFormat & fmt, std::shared_ptr<CloudList> &cl,
             std::shared_ptr<LayerList> &ll, QWidget *parent = 0);
    ~GLWidget();

    void setGLD(std::shared_ptr<GLData> gld);
    QSize minimumSizeHint() const;
    QSize sizeHint() const;

	void resetRotationMatrix();

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

 private:
    std::shared_ptr<CloudList> cl_;
    std::shared_ptr<LayerList> ll_;
    std::shared_ptr<GLData> gld_;

    Camera camera_;
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

