#include "glwidget.h"
#include "qapplication.h"
#include <cstdio>

GLWidget::GLWidget(QWidget* parent )
    : QGLWidget( parent ),
      m_vertexBuffer( QGLBuffer::VertexBuffer )
{
    qApp->installEventFilter(this);
    app_data = AppData::Instance();
    aspectRatio = 1.0f;

    
    modelview_mat = glm::mat4(1.0f);
    offsetVec = glm::vec4(0.0f,0.0f,0.0f,0.0f);

    float zNear = 0.1f; float zFar = 100.0f;

    cameraToClipMatrix = glm::perspective(35.0f, aspectRatio, zNear, zFar);


    glFormat.setVersion( 3, 3 );
    //glFormat.setProfile( QGLFormat::CoreProfile ); // Requires >=Qt-4.8.0
    glFormat.setSampleBuffers( true );
    glFormat.setDoubleBuffer( true );
    glFormat.setDepth( true );
    glFormat.setStencil( true );
    glFormat.setAlpha( true );
    QGLWidget(glFormat, parent);

    setAutoBufferSwap ( true );
    setMouseTracking( true );


    //glm::vec3(app_data->cloud->sensor_origin_(0), app_data->cloud->sensor_origin_(1), app_data->cloud->sensor_origin_(2)),
    //glm::fquat( (float)app_data->cloud->sensor_orientation_.x(), app_data->cloud->sensor_orientation_.y(), app_data->cloud->sensor_orientation_.z(), app_data->cloud->sensor_orientation_.w()),

    // View/Object Setup
    glutil::ViewData initialViewData =
    {           
        glm::vec3(app_data->cloud->sensor_origin_(0), app_data->cloud->sensor_origin_(1), app_data->cloud->sensor_origin_(2)), ///<The starting target position position.
        glm::fquat( (float)app_data->cloud->sensor_orientation_.x(), app_data->cloud->sensor_orientation_.y(), app_data->cloud->sensor_orientation_.z(), app_data->cloud->sensor_orientation_.w()), ///<The initial orientation aroudn the target position.
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
        glm::vec3(app_data->cloud->sensor_origin_(0), app_data->cloud->sensor_origin_(1), app_data->cloud->sensor_origin_(2)), ///<The world-space position of the object.
        glm::fquat(0.0f, 0.0f, 0.0f, 0.0f), ///<The world-space orientation of the object.
        //glm::vec3(app_data->cloud->sensor_origin_(0), app_data->cloud->sensor_origin_(1), app_data->cloud->sensor_origin_(2)),
        //glm::fquat( (float)app_data->cloud->sensor_orientation_.x(), app_data->cloud->sensor_orientation_.y(), app_data->cloud->sensor_orientation_.z(), app_data->cloud->sensor_orientation_.w()),
    };

    viewPole = boost::shared_ptr<glutil::ViewPole>(new glutil::ViewPole(initialViewData, viewScale, glutil::MB_RIGHT_BTN));
    objtPole = boost::shared_ptr<glutil::ObjectPole>(new glutil::ObjectPole(initialObjectData, 90.0f/250.0f, glutil::MB_LEFT_BTN, &*viewPole));

}

void GLWidget::initializeGL()
{

    QGLFormat glFormat = QGLWidget::format();
    if ( !glFormat.sampleBuffers() )
        qWarning() << "Could not enable sample buffers";

    // Set the clear color to black
    glClearColor( 0.1f, 0.1f, 0.1f, 1.0f );

    // Prepare a complete shader program...
    if ( !prepareShaderProgram( "shaders/simple.vert", "shaders/simple.frag" ) )
        return;

    m_vertexBuffer.create();
    m_vertexBuffer.setUsagePattern( QGLBuffer::DynamicDraw );
    if ( !m_vertexBuffer.bind() )
    {
        qWarning() << "Could not bind vertex buffer to the context";
        return;
    }
    m_vertexBuffer.allocate(app_data->cloud->points.size() * sizeof(float) * 4);

    // Bind the shader program so that we can associate variables from
    // our application to the shaders
    if ( !m_shader.bind() )
    {
        qWarning() << "Could not bind shader program to context";
        return;
    }

    float data[4];

    for (int i = 0; i < (int)app_data->cloud->size(); i++)
    {
        data[0] = app_data->cloud->points[i].x;
        data[1] = app_data->cloud->points[i].y;
        data[2] = app_data->cloud->points[i].z;
        data[3] = app_data->cloud->points[i].intensity;

        //std::printf("point (%f, %f %f)\n", app_data->cloud->points[i].x, app_data->cloud->points[i].y, app_data->cloud->points[i].z);

        int offset = 4*sizeof(float)*i;
        m_vertexBuffer.write(offset, reinterpret_cast<const void *> (data), sizeof(data));
    }


    // Enable the "vertex" attribute to bind it to our currently bound
    // vertex buffer.
    m_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
    m_shader.enableAttributeArray( "vertex" );


    glUniformMatrix4fv(m_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, glm::value_ptr(cameraToClipMatrix));
    glUniformMatrix4fv(m_shader.uniformLocation("modelToCameraMatrix"), 1, GL_FALSE, glm::value_ptr(modelview_mat));
}

bool GLWidget::prepareShaderProgram( const QString& vertexShaderPath,
                                     const QString& fragmentShaderPath )
{
    // First we load and compile the vertex shader...
    bool result = m_shader.addShaderFromSourceFile( QGLShader::Vertex, vertexShaderPath );
    if ( !result )
        qWarning() << m_shader.log();

    // ...now the fragment shader...
    result = m_shader.addShaderFromSourceFile( QGLShader::Fragment, fragmentShaderPath );
    if ( !result )
        qWarning() << m_shader.log();

    // ...and finally we link them to resolve any references.
    result = m_shader.link();
    if ( !result )
        qWarning() << "Could not link shader program:" << m_shader.log();

    return result;
}

void GLWidget::resizeGL( int w, int h )
{
    // Set the viewport to window dimensions
    cameraToClipMatrix[0].x = aspectRatio * (h / (float)w);
    cameraToClipMatrix[1].y = aspectRatio;
    glUniformMatrix4fv(m_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, glm::value_ptr(cameraToClipMatrix));
    glViewport( 0, 0, w, qMax( h, 1 ) );
    updateGL();
}

void GLWidget::paintGL()
{
    // Clear the buffer with the current clearing color
    glClearDepth(1.0f);
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Calculate modelview matrix
    glm::mat4 translate(1.0f);
    translate[3]+= offsetVec;
    modelview_mat = viewPole->CalcMatrix() * translate * objtPole->CalcMatrix();

/*
    glm::mat4 tmp = viewPole->CalcMatrix();
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].x, tmp[1].x, tmp[2].x, tmp[3].x);
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].y, tmp[1].y, tmp[2].y, tmp[3].y);
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].z, tmp[1].z, tmp[2].z, tmp[3].z);
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].w, tmp[1].w, tmp[2].w, tmp[3].w);
    printf("\n\n");

    tmp = translate;
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].x, tmp[1].x, tmp[2].x, tmp[3].x);
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].y, tmp[1].y, tmp[2].y, tmp[3].y);
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].z, tmp[1].z, tmp[2].z, tmp[3].z);
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].w, tmp[1].w, tmp[2].w, tmp[3].w);
    printf("\n\n");

    tmp = objtPole->CalcMatrix();
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].x, tmp[1].x, tmp[2].x, tmp[3].x);
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].y, tmp[1].y, tmp[2].y, tmp[3].y);
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].z, tmp[1].z, tmp[2].z, tmp[3].z);
    std::printf("%f,\t %f,\t %f,\t %f \n", tmp[0].w, tmp[1].w, tmp[2].w, tmp[3].w);
    printf("\n\n");
*/
    // Set new modelview matrix
    glUniformMatrix4fv(m_shader.uniformLocation("modelToCameraMatrix"), 1, GL_FALSE, glm::value_ptr(modelview_mat));

    // Draw stuff
    glDrawArrays( GL_POINTS, 0, app_data->cloud->size() );
}

void GLWidget::mouseDoubleClickEvent ( QMouseEvent * event ){

}

void GLWidget::mouseMoveEvent ( QMouseEvent * event ){

    if(mouseDown == Qt::RightButton)
        viewPole->MouseMove(glm::ivec2(event->x(), event->y()));
    if(mouseDown == Qt::LeftButton)
        objtPole->MouseMove(glm::ivec2(event->x(), event->y()));

    if(mouseDown)
        mouseDrag = true;

    updateGL();
}

void GLWidget::mousePressEvent ( QMouseEvent * event ){
    mouseDrag = false;
    mouseDown = event->button();

    if(event->button() == Qt::RightButton)
        viewPole->MouseClick(glutil::MB_RIGHT_BTN, true, 0, glm::ivec2(event->x(), event->y()));

    else if (event->button() == Qt::LeftButton)
        objtPole->MouseClick(glutil::MB_LEFT_BTN, true, 0, glm::ivec2(event->x(), event->y()));

    updateGL();
}

void GLWidget::mouseReleaseEvent ( QMouseEvent * event ){

    objtPole->MouseClick(glutil::MB_LEFT_BTN, false, 0, glm::ivec2(event->x(), event->y()));
    viewPole->MouseClick(glutil::MB_RIGHT_BTN, false, 0, glm::ivec2(event->x(), event->y()));

    mouseDrag = false;
    mouseDown = Qt::NoButton;
    updateGL();
}

void GLWidget::wheelEvent ( QWheelEvent * event ){
    viewPole->MouseWheel(event->delta(), 0, glm::ivec2(event->x(), event->y()));
    updateGL();
}

void GLWidget::keyPressEvent ( QKeyEvent * event ){
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
            //offsetVec = offsetVec + inv*glm::vec4(0.5f, 0.0f, 0.0f, 0.0f);
            viewPole->CharPress('d');
            break;
    case Qt::Key_A:
    case Qt::Key_Left:
            //offsetVec = offsetVec + inv*glm::vec4(-0.5f, 0.0f, 0.0f, 0.0f);
            viewPole->CharPress('a');
            break;
    case Qt::Key_W:
    case Qt::Key_Up:
            //offsetVec = offsetVec + inv*glm::vec4(0.0f, 0.5f, 0.f, 0.0f);
            viewPole->CharPress('w');
            break;
    case Qt::Key_S:
    case Qt::Key_Down:
            //offsetVec = offsetVec + inv*glm::vec4(0.0f, -0.5f, 0.0f, 0.0f);
            viewPole->CharPress('s');
            break;
    case Qt::Key_Q:
            viewPole->CharPress('q');
            break;
    case Qt::Key_E:
            viewPole->CharPress('e');
            break;


    /*case 'f': // flush stats
            stats.clear();
            for (int j = 0; j < 33; ++j){
                    mean.histogram[j] = 0.0f;
                    mean.histogram[j] = 0.0f;
            }
            break;
    case 'q': // toggle flood fill
            filling = !filling;
            sampling = false;
            break;

    case 'e': // toggle sample
            sampling = !sampling;
            filling = false;
            break;
    case '+':
            if(vals_in_range < 33)
                    vals_in_range++;
            printf("%d\n", vals_in_range);
            break;
    case '-':
            if(vals_in_range > 0)
                    vals_in_range--;
            printf("%d\n", vals_in_range);
            break;
    */

    }
    updateGL();

}

 bool GLWidget::eventFilter(QObject *object, QEvent *event)
 {
     if (event->type() == QEvent::KeyPress) {
         QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
         keyPressEvent (keyEvent);
         return true;
     }
     return false;
 }

 void GLWidget::reloadCloud(){
    m_vertexBuffer.allocate(app_data->cloud->points.size() * sizeof(float) * 4);
    float data[4];
    for (int i = 0; i < (int)app_data->cloud->size(); i++)
    {
        data[0] = app_data->cloud->points[i].x;
        data[1] = app_data->cloud->points[i].y;
        data[2] = app_data->cloud->points[i].z;
        data[3] = app_data->cloud->points[i].intensity;

        //std::printf("point (%f, %f %f)\n", app_data->cloud->points[i].x, app_data->cloud->points[i].y, app_data->cloud->points[i].z);
        int offset = 4*sizeof(float)*i;
        m_vertexBuffer.write(offset, reinterpret_cast<const void *> (data), sizeof(data));
    }
    std::printf("Reloaded\n");
 }