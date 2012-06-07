#include "glwidget.h"
#include "qapplication.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>

GLWidget::GLWidget(QWidget* parent )
    : QGLWidget( parent ),
      point_buffer( QGLBuffer::VertexBuffer )
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
    start_x = 0;
    start_y = 0;

    glFormat.setVersion( 3, 3 );
    glFormat.setProfile( QGLFormat::CoreProfile ); // Requires >=Qt-4.8.0
    glFormat.setSampleBuffers( true );
    glFormat.setDoubleBuffer( true );
    glFormat.setDepth( true );
    glFormat.setStencil( true );
    glFormat.setAlpha( true );

    if ( !glFormat.sampleBuffers() )
        qWarning() << "Could not enable sample buffers";

    QGLWidget(glFormat, parent);

    setAutoBufferSwap ( true );
    setMouseTracking( true );

    // overlay painting related
    setAutoFillBackground(false);

    // OpenCL
    clGetPlatformIDs(1, &platform, NULL);


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
    };

    viewPole = boost::shared_ptr<glutil::ViewPole>(new glutil::ViewPole(initialViewData, viewScale, glutil::MB_RIGHT_BTN));
    objtPole = boost::shared_ptr<glutil::ObjectPole>(new glutil::ObjectPole(initialObjectData, 90.0f/250.0f, glutil::MB_LEFT_BTN, &*viewPole));

}

void GLWidget::initializeGL()
{
    // Set the clear color to black
    glClearColor( 0.1f, 0.1f, 0.1f, 1.0f );

    // Prepare a complete shader program...
    if ( !prepareShaderProgram(point_shader, "shaders/points.vert", "shaders/points.frag" ) )
        return;

    point_buffer.create();
    point_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
    if ( !point_buffer.bind() )
    {
        qWarning() << "Could not bind vertex buffer to the context";
        return;
    }
    point_buffer.allocate(app_data->cloud->points.size() * sizeof(float) * 4);

    // Bind the shader program so that we can associate variables from our application to the shaders
    if ( !point_shader.bind() )
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
        point_buffer.write(offset, reinterpret_cast<const void *> (data), sizeof(data));
    }


    // Enable the "vertex" attribute to bind it to our currently bound
    // vertex buffer.
    point_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
    point_shader.enableAttributeArray( "vertex" );


    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, glm::value_ptr(cameraToClipMatrix));
    glUniformMatrix4fv(point_shader.uniformLocation("modelToCameraMatrix"), 1, GL_FALSE, glm::value_ptr(modelview_mat));
    
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
    
    p_vbocl = clCreateFromGLBuffer(context, CL_MEM_WRITE_ONLY, point_buffer.bufferId(), NULL);

    // For convenience use C++ to load the program source into memory
    std::ifstream file("dim.cl");
    std::string prog(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));
    file.close();
    const char* source = prog.c_str();
    const size_t kernelsize = prog.length()+1;
    program = clCreateProgramWithSource(context, 1, (const char**) &source,
                                 &kernelsize, NULL);

    // Build the program executable
    int err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (err != CL_SUCCESS) {
        size_t len;
        char buffer[2048];

        std::cerr << "Error: Failed to build program executable!" << endl;
        clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG,
                           sizeof(buffer), buffer, &len);
        std::cerr << buffer << endl;
        exit(1);
    }

    // Create the compute kernel in the program
    kernel = clCreateKernel(program, "dim", &err);
        if (!kernel || err != CL_SUCCESS) {
        std::cerr << "Error: Failed to create compute kernel!" << endl;
        exit(1);
    }

    // Set the kernel argument
    clSetKernelArg(kernel, 0, sizeof(cl_mem), (void*)&p_vbocl);

}

bool GLWidget::prepareShaderProgram(QGLShaderProgram & shader, const QString& vertexShaderPath, const QString& fragmentShaderPath )
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
        qWarning() << "Could not link shader program:" << point_shader.log();

    return result;
}

void GLWidget::resizeGL( int w, int h )
{
    // Set the viewport to window dimensions
    cameraToClipMatrix[0].x = aspectRatio * (h / (float)w);
    cameraToClipMatrix[1].y = aspectRatio;
    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, glm::value_ptr(cameraToClipMatrix));
    glViewport( 0, 0, w, qMax( h, 1 ) );
    updateGL();
}

void GLWidget::paintGL(){
    update();
}

void GLWidget::paintEvent(QPaintEvent *event)
{

    Q_UNUSED(event);
    anim+= 1.0f;

    // map OpenGL buffer object for writing from OpenCL
    glFinish();
    clEnqueueAcquireGLObjects(cmd_queue, 1, &p_vbocl, 0,0,0);

    // Set queue the kernel
    clSetKernelArg(kernel, 1, sizeof(float), (void*)&anim);
    const size_t buffsize = app_data->cloud->points.size();
    clEnqueueNDRangeKernel(cmd_queue, kernel, 1, NULL, &buffsize, NULL, 0, 0, 0);

    // queue unmap buffer object
    clEnqueueReleaseGLObjects(cmd_queue, 1, &p_vbocl, 0,0,0);
    clFinish(cmd_queue);




    // Clear the buffer with the current clearing color
    glClearDepth(1.0f);
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Calculate modelview matrix
    glm::mat4 translate(1.0f);
    translate[3]+= offsetVec;
    modelview_mat = viewPole->CalcMatrix() * translate * objtPole->CalcMatrix();

    // Set new modelview matrix
    glUniformMatrix4fv(point_shader.uniformLocation("modelToCameraMatrix"), 1, GL_FALSE, glm::value_ptr(modelview_mat));

    point_buffer.bind();

    // Draw points
    glDrawArrays( GL_POINTS, 0, app_data->cloud->size());

    // Overlay drawing
    glDrawArrays( GL_LINES, 0, app_data->cloud->size());

    // TODO Orthogonal modelview
    // TODO Manual painting
    // TODO Figure out indexed painting

    int * lasso_data = &(lasso[0].x());
    unsigned int lasso_index[lasso.size()*2];
    for(int i = 0; i < lasso.size(); i++){
        lasso_index[i*2] = i*2;
        lasso_index[i*2+1] = i*2+1;
    }

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(lasso.size(), GL_INT, 0, lasso_data);
    glDrawElements(GL_LINE, lasso.size()+1, GL_UNSIGNED_INT, lasso_index);
    glDisableClientState(GL_VERTEX_ARRAY);


    /*glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    drawLasso(&painter);
    painter.end();

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    */
}

void GLWidget::drawLasso(QPainter *painter)
 {
    if(lasso.size()<2){
        return;
    }

    painter->setPen(Qt::green);
    for(int i = 0; i < lasso.size()-2; i++){
        painter-> drawLine (lasso[i].x(), lasso[i].y(), lasso[i+1].x(), lasso[i+1].y());
    }

 }

void GLWidget::addLassoPoint(int x, int y){
    if(lasso.empty()){
        lasso.push_back(Eigen::Vector2i(x,y));
    }
    else{
        Eigen::Vector2i end = lasso.back();
        lasso.back() = Eigen::Vector2i(x, y);
        lasso.push_back(end);
    }
}

void GLWidget::moveLasso(int x, int y){
    if(!lasso.empty()){
        lasso.back() = Eigen::Vector2i(x, y);
    }
}


void GLWidget::mouseDoubleClickEvent ( QMouseEvent * event ){
    // End lasso
}

void GLWidget::mouseMoveEvent ( QMouseEvent * event ){

    if(mouseDown == Qt::RightButton)
        viewPole->MouseMove(glm::ivec2(event->x(), event->y()));
    if(mouseDown == Qt::LeftButton)
        objtPole->MouseMove(glm::ivec2(event->x(), event->y()));

    if(sqrt(pow(start_x-event->x(),2) + pow(start_x-event->x(),2)) > 5)
        moved = true;

    if(moved)
        moveLasso(event->x(), event->y());

    updateGL();
}

void GLWidget::mousePressEvent ( QMouseEvent * event ){
    mouseDown = event->button();
    start_x = 0;
    start_y = 0;
    moved = false;

    if(event->button() == Qt::RightButton)
        viewPole->MouseClick(glutil::MB_RIGHT_BTN, true, 0, glm::ivec2(event->x(), event->y()));

    else if (event->button() == Qt::LeftButton){
        objtPole->MouseClick(glutil::MB_LEFT_BTN, true, 0, glm::ivec2(event->x(), event->y()));
        addLassoPoint(event->x(), event->y());
    }

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

    // BS not found
    if(app_data->fpfhs->points[min_index].histogram[0] != 0){
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
    }
    else{
        printf("bs found\n");
    }

    // Quick filling

    if(filling){
        printf("filling!!\n");
        std::vector<bool> filled(app_data->cloud->points.size(), false);
        std::queue<int> myqueue;

        myqueue.push(min_index);

        int current;

        //glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
        float empty[4] = {0.0f, 0.0f, 0.0f, 0.0f};

        int count = 0; // Stops ape shit

        while (!myqueue.empty() && count++ < 10000){
            current = myqueue.front(); myqueue.pop();

            // fill
            // Update buffer
            int offset = 4*sizeof(float)*current;
            point_buffer.write(offset, reinterpret_cast<const void *> (empty), sizeof(empty));

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

                myqueue.push(idx);
            }

        }
        //glBindBuffer(GL_ARRAY_BUFFER, 0);
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
     Q_UNUSED(object);
     if (event->type() == QEvent::KeyPress ) {
         QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
         keyPressEvent (keyEvent);
         return true;
     }
     return false;
 }

 void GLWidget::reloadCloud(){
    point_buffer.allocate(app_data->cloud->points.size() * sizeof(float) * 4);
    float data[4];
    for (int i = 0; i < (int)app_data->cloud->size(); i++)
    {
        data[0] = app_data->cloud->points[i].x;
        data[1] = app_data->cloud->points[i].y;
        data[2] = app_data->cloud->points[i].z;
        data[3] = app_data->cloud->points[i].intensity;

        //std::printf("point (%f, %f %f)\n", app_data->cloud->points[i].x, app_data->cloud->points[i].y, app_data->cloud->points[i].z);
        int offset = 4*sizeof(float)*i;
        point_buffer.write(offset, reinterpret_cast<const void *> (data), sizeof(data));
    }
    std::printf("Reloaded\n");
 }
