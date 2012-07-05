#include "glarea.h"
#include "qapplication.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <Eigen/Dense>
#include "utilities.h"
#include <time.h>
#include <stdlib.h>
#include "interfaces.h"

GLArea::GLArea(QWidget* parent )
    //: QGLWidget(QGLFormat(QGL::HasOverlay)),
    : QGLWidget(parent)
{
    qApp->installEventFilter(this);
    cm = CloudModel::Instance();

    activeEditPlugin = NULL;

    aspectRatio = 1.0f;

    //TODO: Move out of here
    filling = false;
    
    modelview_mat = glm::mat4(1.0f);
    offsetVec = glm::vec4(0.0f,0.0f,0.0f,0.0f);

    float zNear = 0.1f; float zFar = 100.0f;
    cameraToClipMatrix = glm::perspective(35.0f, aspectRatio, zNear, zFar);

    moved = false;
    start_x = 0;
    start_y = 0;

    glFormat.setVersion( 3, 3 );
    glFormat.setProfile( QGLFormat::CoreProfile );
    glFormat.setSampleBuffers( true );
    glFormat.setDoubleBuffer( true );
    glFormat.setDepth( true );
    glFormat.setAlpha( true );

    if ( !glFormat.sampleBuffers() )
        qWarning() << "Could not enable sample buffers";

    QGLWidget(glFormat, parent);

    setAutoBufferSwap ( true );
    setMouseTracking( true );

    // overlay painting related
    setAutoFillBackground(false);    // OpenCL
    clGetPlatformIDs(1, &platform, NULL);


    // View/Object Setup
    glutil::ViewData initialViewData =
    {           
        glm::vec3(cm->cloud->sensor_origin_(0), cm->cloud->sensor_origin_(1), cm->cloud->sensor_origin_(2)), ///<The starting target position position.
        glm::fquat( (float)cm->cloud->sensor_orientation_.x(), cm->cloud->sensor_orientation_.y(), cm->cloud->sensor_orientation_.z(), cm->cloud->sensor_orientation_.w()), ///<The initial orientation aroudn the target position.
        5.0f,   ////<The initial radius of the camera from the target point.
        0.0f    ///<The initial spin rotation of the "up" axis, relative to \a orient
    };

    glutil::ViewScale viewScale =
    {
        -140.0f, 140.0f, // View radius min & max
        0.2f, 0.1f,     // Rotation offsets
        0.2f, 0.1f,		// Movement offsets
        90.0f/250.0f    // Degrees to rotate per pixel moved
    };

    glutil::ObjectData initialObjectData =
    {
        glm::vec3(cm->cloud->sensor_origin_(0), cm->cloud->sensor_origin_(1), cm->cloud->sensor_origin_(2)), ///<The world-space position of the object.
        glm::fquat(0.0f, 0.0f, 0.0f, 0.0f), ///<The world-space orientation of the object.
    };

    viewPole = boost::shared_ptr<glutil::ViewPole>(new glutil::ViewPole(initialViewData, viewScale, glutil::MB_RIGHT_BTN));
    objtPole = boost::shared_ptr<glutil::ObjectPole>(new glutil::ObjectPole(initialObjectData, 90.0f/250.0f, glutil::MB_LEFT_BTN, &*viewPole));

    srand (time(NULL));
}

int GLArea::test(){
    return 3;
}

void GLArea::initializeGL()
{
    // Set the clear color to black
    glClearColor( 0.1f, 0.1f, 0.1f, 1.0f );

    glEnable(GL_DEPTH_TEST);

    // Point shader and buffers
    if ( !prepareShaderProgram(point_shader, ":/shaders/points.vert", ":/shaders/points.frag" ) )
        return;

    if ( !point_shader.bind() )
    {
        qWarning() << "Could not bind shader program to context";
        return;
    }
    point_shader.enableAttributeArray( "vertex" );
    point_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );

    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, glm::value_ptr(cameraToClipMatrix));
    glError("121");


    // Create all buffers here

    printf("size in kb: %f\n", (cm->cloud->size() * 4)/(1024.0f));
    glError("134");

    point_shader.release();


    // Setup OpenCL
    cl_int result = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);

    if(result != CL_SUCCESS)
        exit(-1);

    GLXContext glCtx = glXGetCurrentContext();

    //http://www.codeproject.com/Articles/201263/Part-6-Primitive-Restart-and-OpenGL-Interoperabili
    cl_context_properties props[] = {
            CL_CONTEXT_PLATFORM, 
            (cl_context_properties)platform,
            CL_GLX_DISPLAY_KHR,
            (intptr_t) glXGetCurrentDisplay(),
            CL_GL_CONTEXT_KHR,
            (intptr_t) glCtx,
            0
    };

    context = clCreateContext(props, 1, &device, NULL, NULL, NULL);
    cmd_queue = clCreateCommandQueue(context, device, 0, NULL);

    if(result != CL_SUCCESS)
        qWarning() << "CL object create failed:" << oclErrorString(result);

}

// Puts mouse in NDC
Eigen::Vector2f GLArea::normalized_mouse(int x, int y){
    return Eigen::Vector2f(x/(width()/2.0f) - 1.0f, -(y/(height()/2.0f) - 1.0f));
}


bool GLArea::prepareShaderProgram(QGLShaderProgram & shader, const QString& vertexShaderPath, const QString& fragmentShaderPath )
{
    // Load and compile the vertex shader
    bool result = shader.addShaderFromSourceFile( QGLShader::Vertex, vertexShaderPath );
    if ( !result )
        qWarning() << shader.log();

    // Load and compile the fragment shader
    result = shader.addShaderFromSourceFile( QGLShader::Fragment, fragmentShaderPath );
    if ( !result )
        qWarning() << shader.log();

    // Link them to resolve any references.
    result = shader.link();
    if ( !result )
        qWarning() << "Could not link shader program:" << shader.log();

    return result;
}

void GLArea::resizeGL( int w, int h )
{
    // Set the viewport to window dimensions
    cameraToClipMatrix[0].x = aspectRatio * (h / (float)w);
    cameraToClipMatrix[1].y = aspectRatio;
    point_shader.bind();
    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, glm::value_ptr(cameraToClipMatrix));
    glError("256");
    glViewport( 0, 0, w, qMax( h, 1 ) );
}

void GLArea::paintGL(){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Calculate modelview matrix
    glm::mat4 translate(1.0f);
    translate[3]+= offsetVec;
    modelview_mat = viewPole->CalcMatrix() * translate * objtPole->CalcMatrix();

    point_shader.bind();
    glUniformMatrix4fv(point_shader.uniformLocation("modelToCameraMatrix"), 1, GL_FALSE, glm::value_ptr(modelview_mat));
    glError("274");


    cm->point_buffer.bind();
    point_shader.enableAttributeArray( "vertex" );
    point_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );

    glEnable(GL_PRIMITIVE_RESTART);
    glPrimitiveRestartIndex((unsigned int)-1);
    glPointSize(5);

    std::vector<Layer> & layers = cm->layerList.layers;

    for(unsigned int i = 0; i < layers.size(); i++){
        Eigen::Vector3f colour(1,1,1);
        if(layers[i].active)
            colour = layers[i].colour;

        glUniform3fv(point_shader.uniformLocation("layerColour"), 1, colour.data());
        glError("285");
        layers[i].gl_index_buffer.bind();
        glDrawElements(GL_POINTS, cm->cloud->size(), GL_UNSIGNED_INT, 0);
        glError("289");
    }

    cm->point_buffer.release();
    point_shader.release();

    if(activeEditPlugin)
        activeEditPlugin->paintGL(cm, this);

}

inline float rand_range(float from, float to){
    return (rand()/(float)RAND_MAX) * (to-from) + from;
}


void GLArea::mouseDoubleClickEvent ( QMouseEvent * event ){
    if(activeEditPlugin && !activeEditPlugin->mouseDoubleClickEvent(event, cm, this))
        return;

}

void GLArea::mouseMoveEvent ( QMouseEvent * event ){

    if(activeEditPlugin && !activeEditPlugin->mouseMoveEvent(event, cm, this))
        return;

    if(mouseDown == Qt::RightButton)
        viewPole->MouseMove(glm::ivec2(event->x(), event->y()));
    if(mouseDown == Qt::LeftButton)
        objtPole->MouseMove(glm::ivec2(event->x(), event->y()));

    if(sqrt(pow(start_x-event->x(),2) + pow(start_x-event->x(),2)) > 5)
        moved = true;

    if(mouseDown)
        updateGL();
}

void GLArea::mousePressEvent ( QMouseEvent * event ){


    if(activeEditPlugin && !activeEditPlugin->mousePressEvent(event, cm, this))
        return;

    mouseDown = event->button();
    start_x = 0;
    start_y = 0;
    moved = false;

    if(event->button() == Qt::RightButton)
        viewPole->MouseClick(glutil::MB_RIGHT_BTN, true, 0, glm::ivec2(event->x(), event->y()));

    else if (event->button() == Qt::LeftButton){
        objtPole->MouseClick(glutil::MB_LEFT_BTN, true, 0, glm::ivec2(event->x(), event->y()));
    }

    updateGL();
}

void GLArea::mouseReleaseEvent ( QMouseEvent * event ){

    if(activeEditPlugin && !activeEditPlugin->mouseReleaseEvent(event, cm, this))
        return;

    objtPole->MouseClick(glutil::MB_LEFT_BTN, false, 0, glm::ivec2(event->x(), event->y()));
    viewPole->MouseClick(glutil::MB_RIGHT_BTN, false, 0, glm::ivec2(event->x(), event->y()));

    if(!moved){
        click(event->x(), event->y());
    }

    updateGL();

    moved = false;
    mouseDown = Qt::NoButton;
}

inline float distance(glm::vec3 x0, glm::vec3 x1, glm::vec3 x2){
    glm::vec3 top = glm::cross(x2-x1, x1-x0);
    glm::vec3 bot = x2-x1;
    return top.x*top.x + top.y*top.y + top.z*top.z / bot.x*bot.x + bot.y*bot.y + bot.z*bot.z;
}

void GLArea::click(int x, int y){

    double mvmatrix[16];
    double projmatrix[16];
    int viewport[4];

    double dX, dY, dZ, dClickY; // glUnProject uses doubles, but I'm using floats for these 3D vectors

    for (int i = 0; i < 16; ++i)
    {
        projmatrix[i] = glm::value_ptr(cameraToClipMatrix)[i];
        mvmatrix[i] = glm::value_ptr(modelview_mat)[i];
    }

    glGetIntegerv(GL_VIEWPORT, viewport);

    dClickY = double (height() - y); // OpenGL renders with (0,0) on bottom, mouse reports with (0,0) on top

    gluUnProject ((double) x, dClickY, 0.0, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    glm::vec3 p1 = glm::vec3 ( (float) dX, (float) dY, (float) dZ );
    gluUnProject ((double) x, dClickY, 1.0f, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    glm::vec3 p2 = glm::vec3 ( (float) dX, (float) dY, (float) dZ );

    int min_index = -1;
    float min_val = FLT_MAX;

    // Can be gpu accelerated!!
    for (unsigned int i = 0; i < cm->cloud->points.size(); i++) {
        if(cm->cloud->points[i].intensity < 0.0001f)
            continue;
        glm::vec3 point = glm::vec3(cm->cloud->points[i].x, cm->cloud->points[i].y, cm->cloud->points[i].z);
        float dist = distance(point, p1, p2);

        if(dist < min_val){
            min_index = i;
            min_val = dist;
        }
    }

    if(min_index == -1){
        printf("Cannot find closest point\n");
        return;
    }

    // point filling
    float empty2[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    cm->point_buffer.write(4*sizeof(float)*min_index, reinterpret_cast<const void *> (empty2), sizeof(empty2));

    // 3d brush
    /*if(filling){
        printf("filling!!\n");
        std::vector<bool> filled(cm->cloud->points.size(), false);
        std::queue<int> myqueue;

        myqueue.push(min_index);

        int current;
        float empty[4] = {0.0f, 0.0f, 0.0f, 0.0f};

        int count = 0; // Stops ape shit

        while (!myqueue.empty() && count++ < 10000){
            current = myqueue.front(); myqueue.pop();

            // fill
            // Update buffer
            int offset = 4*sizeof(float)*current;
            cm->point_buffer.write(offset, reinterpret_cast<const void *> (empty), sizeof(empty));

            filled[current] = true;

            int idx;

            int K = 20;

            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            cm->kdtree->nearestKSearch (cm->cloud->points[current], K, pointIdxNKNSearch, pointNKNSquaredDistance);


            // push neighbours
            for (int i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                idx = pointIdxNKNSearch[i];

                // Skip NaN points
                if (cm->cloud->points[idx].x != cm->cloud->points[idx].x)
                    continue;

                // skip already filled points
                if (filled[idx])
                    continue;

                myqueue.push(idx);
            }

        }
    }*/

}

void GLArea::wheelEvent ( QWheelEvent * event ){

    if(activeEditPlugin && !activeEditPlugin->wheelEvent(event, cm, this))
        return;

    viewPole->MouseWheel(event->delta(), 0, glm::ivec2(event->x(), event->y()));
    updateGL();
}

void GLArea::keyPressEvent ( QKeyEvent * event ){

    if(activeEditPlugin && !activeEditPlugin->keyPressEvent(event, cm, this))
        return;

    // Set up inverse rotation
    glm::mat4 inv = glm::transpose(viewPole->CalcMatrix());
    inv[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    inv[0].w = 0.0f;
    inv[1].w = 0.0f;
    inv[2].w = 0.0f;

    switch (event->key())
    {
    case Qt::Key_Escape:
            QCoreApplication::instance()->quit();
            break;
    case Qt::Key_D:
    case Qt::Key_Right:
            viewPole->CharPress('d');
            break;
    case Qt::Key_A:
    case Qt::Key_Left:
            viewPole->CharPress('a');
            break;
    case Qt::Key_W:
    case Qt::Key_Up:
            viewPole->CharPress('w');
            break;
    case Qt::Key_S:
    case Qt::Key_Down:
            viewPole->CharPress('s');
            break;
    case Qt::Key_Q:
            viewPole->CharPress('q');
            break;
    case Qt::Key_E:
            viewPole->CharPress('e');
            break;
    }
    updateGL();

}

 bool GLArea::eventFilter(QObject *object, QEvent *event)
 {
     Q_UNUSED(object);
     if (event->type() == QEvent::KeyPress ) {
         QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
         keyPressEvent (keyEvent);
         return true;
     }
     return false;
 }
