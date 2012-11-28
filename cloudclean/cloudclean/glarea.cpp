#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <time.h>
#include <stdlib.h>

#include <QResource>

#include <Eigen/Dense>
#include <QTime>
#include <QFont>

#include "glarea.h"
#include "qapplication.h"
#include "interfaces.h"
#include "utilities.h"
#include "pluginmanager.h"

GLArea::GLArea(QWidget* parent, PluginManager *pm, CloudModel *cm)
    : QGLWidget(parent)
{
    qApp->installEventFilter(this);
    this->cm = cm;
    this->pm = pm;
    pp = nullptr;

    cfps=0;
    lastTime=0;
    
    camera.setDepthRange(0.1f, 100.0f);
    camera.setAspect(this->width() / float(this->height()));

    moved = false;
    start_move_x = 0;
    start_move_y = 0;
    point_size = 2;

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


    setAutoFillBackground(false);    // OpenCL
    clGetPlatformIDs(1, &platform, NULL);

    mouseDown = Qt::NoButton;

    setAutoBufferSwap(false);

    srand (time(NULL));
}

void GLArea::initializeGL()
{
    glClearColor( 0.9f, 0.9f, 0.9f, 1.0f );
    glEnable(GL_DEPTH_TEST);

    assert(prepareShaderProgram(point_shader, ":/shaders/points.vert", ":/shaders/points.frag", "" ) );

    qDebug("Is it linked? %s", point_shader.isLinked() ? "yes" : "no");

    if ( !point_shader.bind() )
    {
        qWarning() << "Could not bind shader program to context";
        assert(false);
    }
    point_shader.enableAttributeArray( "vertex" );
    point_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, camera.projectionMatrix().data());
    glError("121");
    point_shader.release();


    // Setup OpenCL
    cl_int result = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);

    if(result != CL_SUCCESS){
        qDebug("OpenCL failed!");
        return;
        //exit(-1);
    }

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

    clcontext = clCreateContext(props, 1, &device, NULL, NULL, NULL);
    cmd_queue = clCreateCommandQueue(clcontext, device, 0, NULL);

    if(result != CL_SUCCESS)
        qWarning() << "CL object create failed:" << oclErrorString(result);

}

// Puts mouse in NDC
Eigen::Vector2f GLArea::normalized_mouse(int x, int y){
    return Eigen::Vector2f(x/(width()/2.0f) - 1.0f, -(y/(height()/2.0f) - 1.0f));
}


bool GLArea::prepareShaderProgram(QGLShaderProgram & shader, const QString& vertexShaderPath, const QString& fragmentShaderPath,  const QString& geometryShaderPath)
{
    bool result = shader.addShaderFromSourceFile( QGLShader::Vertex, vertexShaderPath );
    if ( !result ){
        qWarning() << shader.log();
        qDebug("NOOO!!");
    }

    result = shader.addShaderFromSourceFile( QGLShader::Fragment, fragmentShaderPath );
    if ( !result ){
        qWarning() << shader.log();
        qDebug("NOOO!!");
    }

    if (geometryShaderPath.length() > 0){
        result = shader.addShaderFromSourceFile( QGLShader::Geometry, geometryShaderPath );
        if ( !result ){
            qWarning() << shader.log();
            qDebug("NOOO!!");
        }
    }

    // Link them to resolve any references.
    result = shader.link();
    if ( !result ){
        qWarning() << "Could not link shader program:" << shader.log();
        qDebug("NOOO!!");
    }

    return result;
}

void GLArea::resizeGL( int w, int h )
{
    camera.setAspect(w / (float)h);
    point_shader.bind();
    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, camera.projectionMatrix().data());
    glError("256");
    glViewport( 0, 0, w, qMax( h, 1 ) );
}

void GLArea::paintGL(){

    QTime time;
    time.start();

    // This is reset by qpainter
    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthFunc(GL_LEQUAL);
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );


    point_shader.bind();

    glUniformMatrix4fv(point_shader.uniformLocation("modelToCameraMatrix"), 1, GL_FALSE, camera.modelviewMatrix().data());
    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, camera.projectionMatrix().data());
    glError("274");

    cm->point_buffer.bind();
    point_shader.enableAttributeArray( "vertex" );
    point_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );

    glEnable(GL_PRIMITIVE_RESTART);
    glPrimitiveRestartIndex((unsigned int)-1);
    glPointSize(point_size);

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

        if(pm->activeVizPlugin)
            pm->activeVizPlugin->paintLayer(i, cm, this);

    }

    cm->point_buffer.release();
    point_shader.release();


    if(pm->activeEditPlugin)
        pm->activeEditPlugin->paintGL(cm, this);

    if(pm->activeVizPlugin)
        pm->activeVizPlugin->paintGL(cm, this);

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
        float cloud_size = (cm->cloud->size() * 4)/(1024.0f);
        col1Text += QString("Cloud size: %1 KB\n").arg(cloud_size, 7,'f',1);

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

    swapBuffers();
    updateFps(time.elapsed());

}

void GLArea::modelReloaded(){
    Eigen::Affine3f orientation;
    Eigen::Matrix3f rot; rot = cm->cloud->sensor_orientation_;
    orientation.linear() = rot;
    orientation.translation() = cm->cloud->sensor_origin_.head(3);
    camera.setObjectOrientationMatrix(orientation);

    if(pp != nullptr)
        delete pp;
    pp = new PointPicker(this,cm, 0);

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
    if(pm->activeEditPlugin && !pm->activeEditPlugin->mouseDoubleClickEvent(event, cm, this))
        return;
}

void GLArea::mouseMoveEvent ( QMouseEvent * event ){

    if(pm->activeEditPlugin && !pm->activeEditPlugin->mouseMoveEvent(event, cm, this))
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

    if(pm->activeEditPlugin && !pm->activeEditPlugin->mousePressEvent(event, cm, this))
        return;

    mouseDown = event->button();
    start_move_x = 0;
    start_move_y = 0;
    moved = false;


    camera.mouseDown(event->x(), event->y(), event->button());

    //updateGL();
}

void GLArea::mouseReleaseEvent ( QMouseEvent * event ){

    if(pm->activeEditPlugin && !pm->activeEditPlugin->mouseReleaseEvent(event, cm, this))
        return;

    if(event->button() == Qt::RightButton)
        camera.mouseRelease(event->x(), event->y());
    else if (event->button() == Qt::LeftButton)
        camera.mouseRelease(event->x(), event->y());

    if(!moved){

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

void GLArea::wheelEvent ( QWheelEvent * event ){

    if(pm->activeEditPlugin && !pm->activeEditPlugin->wheelEvent(event, cm, this))
        return;

    camera.mouseWheel(event->delta());
    updateGL();
}

void GLArea::keyPressEvent ( QKeyEvent * event ){

    if(pm->activeEditPlugin && !pm->activeEditPlugin->keyPressEvent(event, cm, this))
        return;

    // Set up inverse rotation

    float offset = 0.4;

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
            camera.adjustPosition(0,0, offset); // forward
            break;
    case Qt::Key_S:
    case Qt::Key_Down:
            camera.adjustPosition(0,0,-offset); // backward
            break;
    case Qt::Key_Q:
            camera.adjustPosition(0,-offset,0); // up
            break;
    case Qt::Key_E:
            camera.adjustPosition(0,offset,0); // down
            break;
    case Qt::Key_R:
            if(event->modifiers() == Qt::ControlModifier)
                camera.setPosition(0,0,0); // down
            break;
    case Qt::Key_Plus:
            if(point_size < 30)
                point_size++;
            break;
    case Qt::Key_Minus:
            if(point_size > 1)
                point_size--;
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
