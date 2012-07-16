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
#include <QTime>
#include <QFont>

GLArea::GLArea(QWidget* parent )
    //: QGLWidget(QGLFormat(QGL::HasOverlay)),
    : QGLWidget(parent)
{
    qApp->installEventFilter(this);
    cm = CloudModel::Instance();

    activeEditPlugin = NULL;
    cfps=0;
    lastTime=0;

    //TODO: Move out of here
    filling = false;
    
    camera.setDepthRange(0.1f, 100.0f);

    moved = false;
    start_move_x = 0;
    start_move_y = 0;

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

    srand (time(NULL));
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

    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, camera.projectionMatrix().data());
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
    camera.setAspect(h / (float)w);
    point_shader.bind();
    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, camera.projectionMatrix().data());
    glError("256");
    glViewport( 0, 0, w, qMax( h, 1 ) );
}

void GLArea::paintGL(){

    QTime time;
    time.start();

    // This is reet by qpainter
    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthFunc(GL_LEQUAL);
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    point_shader.bind();

    glUniformMatrix4fv(point_shader.uniformLocation("modelToCameraMatrix"), 1, GL_FALSE, camera.modelviewMatrix().data());
    glError("274");

    cm->point_buffer.bind();
    point_shader.enableAttributeArray( "vertex" );
    point_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );

    glEnable(GL_PRIMITIVE_RESTART);
    glPrimitiveRestartIndex((unsigned int)-1);
    glPointSize(5);

    std::vector<Layer> & layers = cm->layerList.layers;

    for(unsigned int i = 0; i < layers.size(); i++){
        if(!layers[i].visible)
            continue;

        Eigen::Vector3f colour(1,1,1);
        if(layers[i].active)
            colour = layers[i].colour;

        glUniform3fv(point_shader.uniformLocation("layerColour"), 1, colour.data());
        glError("285");
        layers[i].gl_index_buffer.bind();
        glDrawElements(GL_POINTS, cm->cloud->size(), GL_UNSIGNED_INT, 0);
        glError("289");
        layers[i].gl_index_buffer.release();
        glError("289");
    }

    cm->point_buffer.release();
    point_shader.release();

    if(activeEditPlugin)
        activeEditPlugin->paintGL(cm, this);

    //////////////////////////////////////

    QPainter painter(this);
    painter.beginNativePainting();

    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.setPen(Qt::white);
    //qFont.setStyleStrategy(QFont::NoAntialias);
    qFont.setFamily("Helvetica");
    qFont.setPixelSize(12);
    painter.setFont(qFont);
    float barHeight = qFont.pixelSize()*5;
    QFontMetrics metrics = QFontMetrics(font());
    int border = qMax(4, metrics.leading());

    QRect Column_0(width()/10, this->height()-barHeight+border, width()/2, this->height()-border);
    QRect Column_1(width()/2 , this->height()-barHeight+border, width()*3/4,   this->height()-border);
    QRect Column_2(width()*3/4 , this->height()-barHeight+border, width(),   this->height()-border);

    QColor logAreaColor(20, 20, 20, 128);

    painter.fillRect(QRect(0, this->height()-barHeight, width(), this->height()), logAreaColor);

    QString col1Text,col0Text;

    if(cm->isLoaded())
    {

        if ((cfps>0) && (cfps<999))
            col1Text += QString("FPS: %1\n").arg(cfps,7,'f',1);
        col1Text += QString("Vertices: %1\n").arg(cm->cloud->size());
        //col1Text += QString("Faces: %1\n").arg(mm()->cm.fn);

        /*
        if(fov>5) col0Text += QString("FOV: %1\n").arg(fov);
        else col0Text += QString("FOV: Ortho\n");
        if ((clipRatioNear!=1) || (clipRatioFar!=1))
            col0Text += QString("Clipping: N:%1 F:%2\n").arg(clipRatioNear,7,'f',1).arg(clipRatioFar,7,'f',1);

        */
        painter.drawText(Column_0, Qt::AlignLeft | Qt::TextWordWrap, col1Text);
        painter.drawText(Column_1, Qt::AlignLeft | Qt::TextWordWrap, col0Text);
        //if(mm()->cm.Tr != Matrix44f::Identity() ) displayMatrix(painter, Column_2);

    }

    //////////////////////////////////////////

    painter.endNativePainting();

    updateFps(time.elapsed());

}

void GLArea::modelReloaded(){
    Eigen::Affine3f orientation;
    Eigen::Matrix3f rot; rot = cm->cloud->sensor_orientation_;
    orientation.linear() = rot;
    orientation.translation() = cm->cloud->sensor_origin_.head(3);
    camera.setObjectOrientationMatrix(orientation);
}

void GLArea::updateFps(float frameTime)
{
    /*float time = 0.5f*frameTime + 0.5*lastTime;
    lastTime = frameTime;

    cfps = 1.0f/(time/1000.0f);
    */

    static float fpsVector[10];
    static int j=0;
    float averageFps=0;
    if (frameTime>0) {
        fpsVector[j]=frameTime;
        j=(j+1) % 10;
    }
    for (int i=0;i<10;i++) averageFps+=fpsVector[i];
    cfps=1000.0f/(averageFps/10);
    lastTime=frameTime;

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
        camera.mouseMove(event->x(), event->y());
    if(mouseDown == Qt::LeftButton)
        camera.mouseMove(event->x(), event->y());

    if(sqrt(pow(start_move_x-event->x(),2) + pow(start_move_y-event->y(),2)) > 5)
        moved = true;

    if(mouseDown)
        updateGL();
}

void GLArea::mousePressEvent ( QMouseEvent * event ){

    if(activeEditPlugin && !activeEditPlugin->mousePressEvent(event, cm, this))
        return;

    mouseDown = event->button();
    start_move_x = 0;
    start_move_y = 0;
    moved = false;


    camera.mouseDown(event->x(), event->y(), event->button());

    //updateGL();
}

void GLArea::mouseReleaseEvent ( QMouseEvent * event ){

    if(activeEditPlugin && !activeEditPlugin->mouseReleaseEvent(event, cm, this))
        return;

    if(event->button() == Qt::RightButton)
        camera.mouseRelease(event->x(), event->y());
    else if (event->button() == Qt::LeftButton)
        camera.mouseRelease(event->x(), event->y());

    if(!moved){
        //click(event->x(), event->y());
    }

    updateGL();

    moved = false;
    mouseDown = Qt::NoButton;
}

inline float distance(Eigen::Vector3f x0, Eigen::Vector3f x1, Eigen::Vector3f x2){
    Eigen::Vector3f top = (x2-x1).cross(x1-x0);
    Eigen::Vector3f bot = x2-x1;
    return top.x()*top.x() + top.y()*top.y() + top.z()*top.z() / bot.x()*bot.x() + bot.y()*bot.y() + bot.z()*bot.z();
}

void GLArea::click(int x, int y){

    double mvmatrix[16];
    double projmatrix[16];
    int viewport[4];

    double dX, dY, dZ, dClickY; // glUnProject uses doubles, but I'm using floats for these 3D vectors

    for (int i = 0; i < 16; ++i)
    {
        projmatrix[i] = camera.projectionMatrix().data()[i];
        mvmatrix[i] = camera.modelviewMatrix().data()[i];
    }

    glGetIntegerv(GL_VIEWPORT, viewport);

    dClickY = double (height() - y); // OpenGL renders with (0,0) on bottom, mouse reports with (0,0) on top

    gluUnProject ((double) x, dClickY, 0.0, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    Eigen::Vector3f p1 = Eigen::Vector3f ( (float) dX, (float) dY, (float) dZ );
    gluUnProject ((double) x, dClickY, 1.0f, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    Eigen::Vector3f p2 = Eigen::Vector3f ( (float) dX, (float) dY, (float) dZ );

    int min_index = -1;
    float min_val = FLT_MAX;

    // Can be gpu accelerated!!
    for (unsigned int i = 0; i < cm->cloud->points.size(); i++) {
        if(cm->cloud->points[i].intensity < 0.0001f)
            continue;
        Eigen::Vector3f point = Eigen::Vector3f(cm->cloud->points[i].x, cm->cloud->points[i].y, cm->cloud->points[i].z);
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

    camera.mouseWheel(event->delta());
    updateGL();
}

void GLArea::keyPressEvent ( QKeyEvent * event ){

    if(activeEditPlugin && !activeEditPlugin->keyPressEvent(event, cm, this))
        return;

    // Set up inverse rotation

    float offset = 0.2;

    switch (event->key())
    {
    case Qt::Key_Escape:
            QCoreApplication::instance()->quit();
            break;
    case Qt::Key_D:
    case Qt::Key_Right:
            camera.adjustPosition(offset,0,0);
            break;
    case Qt::Key_A:
    case Qt::Key_Left:
            camera.adjustPosition(-offset,0,0);
            break;
    case Qt::Key_W:
    case Qt::Key_Up:
            camera.adjustPosition(0,-offset,0);
            break;
    case Qt::Key_S:
    case Qt::Key_Down:
            camera.adjustPosition(0,offset,0);
            break;
    case Qt::Key_Q:
            camera.adjustPosition(0,0, offset);
            break;
    case Qt::Key_E:
            camera.adjustPosition(0,0,-offset);
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
