#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "glheaders.h"
#include <unordered_map>
#include <memory>
#include <functional>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include "gui/camera.h"
#include "model/cloudlist.h"
#include "model/layerlist.h"

class CloudGLData : public QObject{
    Q_OBJECT
 public:
    CloudGLData(std::shared_ptr<PointCloud> pc);
    ~CloudGLData();
    void draw();

    void copyCloud();
    void copyLabels();
    void copyFlags();

 public slots:
    void syncCloud();
    void syncLabels();
    void syncFlags();

 public:
    std::shared_ptr<PointCloud> pc_;
    GLuint vao_;
    std::shared_ptr<QGLBuffer> label_buffer_;
    std::shared_ptr<QGLBuffer> point_buffer_;
    std::shared_ptr<QGLBuffer> flag_buffer_;

 private:
    bool dirty_labels;
    bool dirty_points;
    bool dirty_flags;
};

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(std::shared_ptr<CloudList> &cl, std::shared_ptr<LayerList> &ll, QWidget *parent = 0);
    ~GLWidget();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

	void resetRotationMatrix();

protected:
    void initializeGL();
    void syncDataModel();
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

 public slots:
   void reloadCloud(std::shared_ptr<PointCloud> cloud);
   void reloadColorLookupBuffer();

 private:
    std::shared_ptr<CloudList> cl_;
    std::shared_ptr<LayerList> ll_;

    Camera camera_;

    QGLShaderProgram program_;
    std::shared_ptr<QGLBuffer> color_lookup_buffer_;

    std::map<std::shared_ptr<PointCloud>, std::shared_ptr<CloudGLData> > cloudgldata_;

    int uni_sampler_;
    int uni_projection_;
    int uni_modelview_;
    int uni_select_color_;

    float selection_color_[4];

    float camera_move_unit_;
    QVector2D mouse_drag_start_;
    float point_render_size_;

    GLuint texture_id_;
};

#endif

