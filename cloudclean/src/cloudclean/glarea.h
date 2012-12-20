#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <ctime>
#include <QtGlobal>

#include "gl_global.h"

#include "cloudclean_global.h"

#include <QGLWidget>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include <QMouseEvent>
#include <QMutex>

#include <GL/glu.h>

#include "cloudmodel.h"
#include "camera.h"
#include "pointpicker.h"

#ifdef Q_WS_X11
    #include <GL/glx.h>
    #undef KeyPress // Defined in X11/X.h, interferes with QEvent::KeyPress
#endif

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include <CL/cl_gl.h>

class EditPluginInterface;
class PluginManager;

class DLLSPEC GLArea : public QGLWidget
{
    Q_OBJECT
public:
    GLArea( QWidget* parent = 0, PluginManager *pm = NULL, CloudModel *cm = NULL);

    bool prepareShaderProgram( QGLShaderProgram& shader,
                               const QString& vertexShaderPath,
                               const QString& fragmentShaderPath, const QString &geometryShaderPath );

    Eigen::Vector2f normalized_mouse(int x, int y);

    void modelReloaded();

protected:
    virtual void initializeGL();
    virtual void resizeGL( int w, int h );
    virtual void paintGL();
    void updateFps(float frameTime);

    void mouseDoubleClickEvent ( QMouseEvent * event );
    void mouseMoveEvent ( QMouseEvent * event );
    void mousePressEvent ( QMouseEvent * event );
    void mouseReleaseEvent ( QMouseEvent * event );
    void wheelEvent ( QWheelEvent * event );
    void keyPressEvent ( QKeyEvent * event );
    bool eventFilter(QObject *object, QEvent *event);

public slots:

private:

    QFont qFont;

    QGLFormat                               glFormat;

    int                                     start_move_x;
    int                                     start_move_y;

    float                                   cfps;
    float                                   lastTime;

    int                                     point_size;

public:
    QGLShaderProgram                        point_shader;

    cl_platform_id                          platform;
    cl_device_id                            device;
    cl_context                              clcontext;
    cl_command_queue                        cmd_queue;

    Qt::MouseButton                         mouseDown;
    bool                                    moved;

    Camera                                  camera;

    CloudModel *                            cm;
    PluginManager *                         pm;
    PointPicker *                           pp;
};

#endif // GLWIDGET_H
