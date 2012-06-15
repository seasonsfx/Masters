#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <ctime>

#define GL3_PROTOTYPES
#include <GL3/gl3.h>
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

#include "appdata.h"
#include "MousePoles.h"

#include <GL/glx.h>
#undef KeyPress // Defined in X11/X.h, interferes with QEvent::KeyPress

class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    GLWidget( QWidget* parent = 0 );

protected:
    virtual void initializeGL();
    virtual void initializeOverlayGL();
    virtual void resizeGL( int w, int h );
    virtual void resizeOverlayGL( int w, int h );
    virtual void paintGL();
    virtual void paintOverlayGL();

    Eigen::Vector2f normalized_mouse(int x, int y);
    void lassoToLayer();
    void addLassoPoint(int x, int y);
    void moveLasso(int x, int y);

    void click(int x, int y);

    void mouseDoubleClickEvent ( QMouseEvent * event );
    void mouseMoveEvent ( QMouseEvent * event );
    void mousePressEvent ( QMouseEvent * event );
    void mouseReleaseEvent ( QMouseEvent * event );
    void wheelEvent ( QWheelEvent * event );
    void keyPressEvent ( QKeyEvent * event );
    bool eventFilter(QObject *object, QEvent *event);

public slots:
    void reloadCloud();

private:
    bool prepareShaderProgram( QGLShaderProgram& shader,
                               const QString& vertexShaderPath,
                               const QString& fragmentShaderPath );

    AppData *               app_data;

    QGLFormat               glFormat;
    QGLShaderProgram        point_shader;
    QGLShaderProgram        lasso_shader;

    QGLBuffer               point_buffer; // Holds point vertices
    std::vector<QGLBuffer>  point_indices; // index buffers that represent layers

    std::vector<Eigen::Vector3f>    point_colours;


    QGLBuffer               lasso_buffer;
    QGLBuffer               lasso_index;

    glm::mat4               cameraToClipMatrix;
    glm::mat4               modelview_mat;
    glm::vec4               offsetVec;
    float                   aspectRatio;

    cl_platform_id          platform;         
    cl_device_id            device;
    cl_context              context;
    cl_command_queue        cmd_queue;
    cl_program              program;
    cl_kernel               kernel;
    size_t                  kernelsize;
    
    cl_mem                  p_vbocl;

    float   anim;

    Qt::MouseButton         mouseDown;
    bool                    moved;
    bool                    lasso_active;
    int                     start_x;
    int                     start_y;

    std::vector<Eigen::Vector2f>   lasso;

    //TODO: Move out of here
    bool                                filling;
    std::vector<pcl::FPFHSignature33>   stats;
    pcl::FPFHSignature33                mean;
    pcl::FPFHSignature33                stdev;

    boost::shared_ptr<glutil::ViewPole>     viewPole;
    boost::shared_ptr<glutil::ObjectPole>   objtPole;
};

#endif // GLWIDGET_H
