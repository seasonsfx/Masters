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

    //TODO: Move out of here
    sampling = false;
    filling = false;
    vals_in_range = 15;
    
    modelview_mat = glm::mat4(1.0f);
    offsetVec = glm::vec4(0.0f,0.0f,0.0f,0.0f);

    float zNear = 0.1f; float zFar = 100.0f;
    cameraToClipMatrix = glm::perspective(35.0f, aspectRatio, zNear, zFar);

    moved = false;

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
        moved = true;

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

    if(!moved){
        printf("CLICK CLICK!!\n");
        clickity(event->x(), event->x());
    }

    moved = false;
    mouseDown = Qt::NoButton;
    updateGL();
}

inline float distance(glm::vec3 x0, glm::vec3 x1, glm::vec3 x2){
    glm::vec3 top = glm::cross(x2-x1, x1-x0);
    glm::vec3 bot = x2-x1;
    return top.x*top.x + top.y*top.y + top.z*top.z / bot.x*bot.x + bot.y*bot.y + bot.z*bot.z;
}

void GLWidget::clickity(int x, int y){
    // This function will find 2 points in world space that are on the line into the screen defined by screen-space( ie. window-space ) point (x,y)

    double mvmatrix[16];
    double projmatrix[16];
    int viewport[4];

    double dX, dY, dZ, dClickY; // glUnProject uses doubles, but I'm using floats for these 3D vectors

    for (int i = 0; i < 16; ++i)
    {
        projmatrix[i] = glm::value_ptr(cameraToClipMatrix)[i];
        mvmatrix[i] = glm::value_ptr(modelview_mat)[i];
        printf("%f ", mvmatrix[i]);
    }
    printf("\n");

    glGetIntegerv(GL_VIEWPORT, viewport);

    dClickY = double (height() - y); // OpenGL renders with (0,0) on bottom, mouse reports with (0,0) on top

    gluUnProject ((double) x, dClickY, 0.0, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    glm::vec3 p1 = glm::vec3 ( (float) dX, (float) dY, (float) dZ );
    gluUnProject ((double) x, dClickY, 0.9, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    glm::vec3 p2 = glm::vec3 ( (float) dX, (float) dY, (float) dZ );

    int min_index = -1;
    float min_val = FLT_MAX;

    // Can be gpu accelerated!!
    for (int i = 0; i < app_data->cloud->points.size(); i++) {
        if(app_data->cloud->points[i].intensity < 0.0001f)
            continue;
        glm::vec3 point = glm::vec3(app_data->cloud->points[i].x, app_data->cloud->points[i].y, app_data->cloud->points[i].z);
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

    // print feature
    printf("Point:\t (%f, %f, %f)\n",
                app_data->cloud->points[min_index].x, app_data->cloud->points[min_index].y, app_data->cloud->points[min_index].z);

    printf("Normal:\t (%f, %f, %f)\n", app_data->normals->points[min_index].data_n[0], app_data->normals->points[min_index].data_n[1], app_data->normals->points[min_index].data_n[2]);

    printf("Feature:\t");
    for (int j = 0; j < 33; ++j)
    {
        printf(", %f ", app_data->fpfhs->points[min_index].histogram[j]);
    }
    printf("\n\n");

    if(sampling){

        if(app_data->fpfhs->points[min_index].histogram[0] == app_data->fpfhs->points[min_index].histogram[0])
                stats.push_back(app_data->fpfhs->points[min_index]);



    ///// STATS!!


        // set initial values
        for (int j = 0; j < 33; ++j){
            mean.histogram[j] = 0.0f;
            mean.histogram[j] = 0.0f;
        }


        //Feature

        // Mean
        for (int i = 0; i < stats.size(); ++i)
        {
            for (int j = 0; j < 33; ++j){
                mean.histogram[j] += stats[i].histogram[j]/stats.size();

            }
        }
        for (int i = 0; i < stats.size(); ++i)
        {
            for (int j = 0; j < 33; ++j){
                stdev.histogram[j] += pow(stats[i].histogram[j]-mean.histogram[j], 2.0f);
            }
        }
        // Final step
        for (int j = 0; j < 33; ++j){
                stdev.histogram[j] = sqrt(stdev.histogram[j]/stats.size());
        }


        printf("Feature mean:\t ");
        // Print stdev
        for (int j = 0; j < 33; ++j)
        {
            printf(", %f ", mean.histogram[j]);
        }
        printf("\n\n");


        printf("Feature stdev:\t ");
        // Print stdev
        for (int j = 0; j < 33; ++j)
        {
            printf(", %f ", stdev.histogram[j]);
        }
        printf("\n\n");

    }

    if(filling && app_data->cloud->points[min_index].x == app_data->cloud->points[min_index].x){
        printf("filling!!\n");
        std::vector<bool> filled(app_data->cloud->points.size(), false);
        std::queue<int> myqueue;

        myqueue.push(min_index);

        int current;

        //glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
        float empty[4] = {0.0f, 0.0f, 0.0f, 0.0f};

        int count; // Stops ape shit

        while (!myqueue.empty() && count++ < 10000){
            current = myqueue.front(); myqueue.pop();

            // fill
            // Update buffer
            glBufferSubData(GL_ARRAY_BUFFER, 4*sizeof(float)*current, 4*sizeof(float), empty);
            filled[current] = true;

            int idx;

            int K = 20;

            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            app_data->kdtree->nearestKSearch (app_data->cloud->points[current], K, pointIdxNKNSearch, pointNKNSquaredDistance);


            // push neighbours
            for (int i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                idx = pointIdxNKNSearch[i];

                // Skip NaN points
                if (app_data->cloud->points[idx].x != app_data->cloud->points[idx].x)
                    continue;

                // skip already filled points
                if (filled[idx])
                    continue;

                int in_range = 0;

                // Check if point is within threshold
                for (int j = 0; j < 33; ++j)
                {
                    if ((app_data->fpfhs->points[idx].histogram[j] - mean.histogram[j]) < stdev.histogram[j])
                    {
                        in_range++;
                    }
                }

                // If histogram in range
                if (in_range > vals_in_range)
                {
                    myqueue.push(idx);
                    //printf("PUSH!!!\n");
                }
            }

        }


        // line
        /*glBindBuffer(GL_ARRAY_BUFFER, lineBufferObject);
        float line[6] = {p1.x, p1.y, p1.z, p2.x, p2.y, p2.z};
        glBufferSubData(GL_ARRAY_BUFFER, 0, 6*sizeof(float), line);
        */
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
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

    case Qt::Key_F: // flush stats
            stats.clear();
            for (int j = 0; j < 33; ++j){
                    mean.histogram[j] = 0.0f;
                    mean.histogram[j] = 0.0f;
            }
            break;
    case Qt::Key_G: // toggle flood fill
            filling = !filling;
            printf("filling toggle\n");
            sampling = false;
            break;

    case Qt::Key_H: // toggle sample
            sampling = !sampling;
            filling = false;
            break;
    case Qt::Key_Plus:
            if(vals_in_range < 33)
                    vals_in_range++;
            printf("%d\n", vals_in_range);
            break;
    case Qt::Key_Minus:
            if(vals_in_range > 0)
                    vals_in_range--;
            printf("%d\n", vals_in_range);
            break;


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
