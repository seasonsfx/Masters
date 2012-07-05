#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <ctime>

#define GL3_PROTOTYPES
#include <../external/gl3.h>
#include <GL/glu.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include <CL/cl_gl.h>

#include <QGLWidget>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include <QMouseEvent>
#include <QMutex>

#include "cloudmodel.h"
#include "MousePoles.h"

#include <GL/glx.h>
#undef KeyPress // Defined in X11/X.h, interferes with QEvent::KeyPress

class EditPluginInterface;

class GLArea : public QGLWidget
{
    Q_OBJECT
public:
    GLArea( QWidget* parent = 0 );

    bool prepareShaderProgram( QGLShaderProgram& shader,
                               const QString& vertexShaderPath,
                               const QString& fragmentShaderPath );

    int test();

    inline Eigen::Vector2f normalized_mouse(int x, int y);
    glm::mat4               cameraToClipMatrix;
    glm::mat4               modelview_mat;

protected:
    virtual void initializeGL();
    virtual void resizeGL( int w, int h );
    virtual void paintGL();

    void click(int x, int y);

    void mouseDoubleClickEvent ( QMouseEvent * event );
    void mouseMoveEvent ( QMouseEvent * event );
    void mousePressEvent ( QMouseEvent * event );
    void mouseReleaseEvent ( QMouseEvent * event );
    void wheelEvent ( QWheelEvent * event );
    void keyPressEvent ( QKeyEvent * event );
    bool eventFilter(QObject *object, QEvent *event);

public slots:

private:

    CloudModel *            cm;

    QGLFormat               glFormat;
    QGLShaderProgram        point_shader;

    glm::vec4               offsetVec;
    float                   aspectRatio;

    
    //cl_mem                  p_vbocl;

    int                     start_x;
    int                     start_y;


    //TODO: Move out of here
    bool                                filling;
    std::vector<pcl::FPFHSignature33>   stats;
    pcl::FPFHSignature33                mean;
    pcl::FPFHSignature33                stdev;

    boost::shared_ptr<glutil::ViewPole>     viewPole;
    boost::shared_ptr<glutil::ObjectPole>   objtPole;

public:
    cl_platform_id          platform;
    cl_device_id            device;
    cl_context              context;
    cl_command_queue        cmd_queue;

    Qt::MouseButton         mouseDown;
    bool                    moved;

    EditPluginInterface * activeEditPlugin;

};

#endif // GLWIDGET_H
