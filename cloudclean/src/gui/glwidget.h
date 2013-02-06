#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "glheaders.h"
#include <memory>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include "gui/camera.h"
#include "model/datamodel.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(std::shared_ptr<DataModel> & dm, QWidget *parent = 0);
    ~GLWidget();

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
    std::shared_ptr<DataModel> dm;

    QVector<QVector4D> vertices_;
    QVector<QVector4D> colours_;
    QVector<int> color_index_;

    Camera camera_;

    QGLShaderProgram program_;
    std::shared_ptr<QGLBuffer> label_colours_;
    std::shared_ptr<QGLBuffer> index_buffer_;  // Used for masking?
    std::shared_ptr<QGLBuffer> vertex_buffer_;

    int attr_vertex_;
    int attr_intensity_;
    int attr_color_index_;
    int uni_sampler_;
    int uni_projection_;
    int uni_modelview_;

    float camera_move_unit;
    QVector2D mouse_drag_start;
    float point_render_size;

    GLuint texture_id_;
    GLuint vao_;

    // So basically here we need qt datastructures that repreent the model state
    // On every draw the data should the model should be checked for modifications
    // Should modifications happen these structure need to be updated

};

#endif

