#include "glwidget.h"
#include "qapplication.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <Eigen/Dense>

// Helper function to get error string
// *********************************************************************
const char* oclErrorString(cl_int error)
{
    static const char* errorString[] = {
        "CL_SUCCESS",
        "CL_DEVICE_NOT_FOUND",
        "CL_DEVICE_NOT_AVAILABLE",
        "CL_COMPILER_NOT_AVAILABLE",
        "CL_MEM_OBJECT_ALLOCATION_FAILURE",
        "CL_OUT_OF_RESOURCES",
        "CL_OUT_OF_HOST_MEMORY",
        "CL_PROFILING_INFO_NOT_AVAILABLE",
        "CL_MEM_COPY_OVERLAP",
        "CL_IMAGE_FORMAT_MISMATCH",
        "CL_IMAGE_FORMAT_NOT_SUPPORTED",
        "CL_BUILD_PROGRAM_FAILURE",
        "CL_MAP_FAILURE",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "CL_INVALID_VALUE",
        "CL_INVALID_DEVICE_TYPE",
        "CL_INVALID_PLATFORM",
        "CL_INVALID_DEVICE",
        "CL_INVALID_CONTEXT",
        "CL_INVALID_QUEUE_PROPERTIES",
        "CL_INVALID_COMMAND_QUEUE",
        "CL_INVALID_HOST_PTR",
        "CL_INVALID_MEM_OBJECT",
        "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR",
        "CL_INVALID_IMAGE_SIZE",
        "CL_INVALID_SAMPLER",
        "CL_INVALID_BINARY",
        "CL_INVALID_BUILD_OPTIONS",
        "CL_INVALID_PROGRAM",
        "CL_INVALID_PROGRAM_EXECUTABLE",
        "CL_INVALID_KERNEL_NAME",
        "CL_INVALID_KERNEL_DEFINITION",
        "CL_INVALID_KERNEL",
        "CL_INVALID_ARG_INDEX",
        "CL_INVALID_ARG_VALUE",
        "CL_INVALID_ARG_SIZE",
        "CL_INVALID_KERNEL_ARGS",
        "CL_INVALID_WORK_DIMENSION",
        "CL_INVALID_WORK_GROUP_SIZE",
        "CL_INVALID_WORK_ITEM_SIZE",
        "CL_INVALID_GLOBAL_OFFSET",
        "CL_INVALID_EVENT_WAIT_LIST",
        "CL_INVALID_EVENT",
        "CL_INVALID_OPERATION",
        "CL_INVALID_GL_OBJECT",
        "CL_INVALID_BUFFER_SIZE",
        "CL_INVALID_MIP_LEVEL",
        "CL_INVALID_GLOBAL_WORK_SIZE",
    };

    const int errorCount = sizeof(errorString) / sizeof(errorString[0]);

    const int index = -error;

    return (index >= 0 && index < errorCount) ? errorString[index] : "";

}

GLWidget::GLWidget(QWidget* parent )
    : QGLWidget( parent ),
      point_buffer( QGLBuffer::VertexBuffer )
{
    qApp->installEventFilter(this);
    app_data = AppData::Instance();
    aspectRatio = 1.0f;

    //TODO: Move out of here
    filling = false;
    lasso_active = false;
    
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
    glFormat.setStencil( true );
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


    // Point shader and buffers
    if ( !prepareShaderProgram(point_shader, "shaders/points.vert", "shaders/points.frag" ) )
        return;
    point_shader.bind();

    //glGenVertexArrays(1, &point_vao);
    //glBindVertexArray(lasso_vao);
    point_buffer.create();
    point_buffer.setUsagePattern( QGLBuffer::DynamicDraw );

    if ( !point_buffer.bind() )
    {
        qWarning() << "Could not bind vertex buffer to the context";
        return;
    }
    point_buffer.allocate(app_data->cloud->points.size() * sizeof(float) * 4);
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
        int offset = 4*sizeof(float)*i;
        point_buffer.write(offset, reinterpret_cast<const void *> (data), sizeof(data));
    }
    point_shader.enableAttributeArray( "vertex" );
    point_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );

    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, glm::value_ptr(cameraToClipMatrix));

    point_buffer.release();
    point_shader.release();

    // Lasso shader and buffered

    if ( !prepareShaderProgram(lasso_shader, "shaders/lasso.vert", "shaders/lasso.frag" ) )
        return;
    lasso_shader.bind();
    lasso_buffer.create();
    lasso_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
    if ( !lasso_buffer.bind() )
    {
        qWarning() << "Could not bind vertex buffer to the context";
        return;
    }
    lasso_buffer.allocate(sizeof(float) * 12);
    if ( !lasso_shader.bind() )
    {
        qWarning() << "Could not bind shader program to context";
        return;
    }
    lasso_shader.enableAttributeArray( "point" );
    lasso_shader.setAttributeBuffer( "point", GL_FLOAT, 0, 2 );


    lasso_shader.release();
    lasso_buffer.release();
    //glBindVertexArray(0);

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

    p_vbocl = clCreateFromGLBuffer(context, CL_MEM_READ_WRITE, point_buffer.bufferId(), &result);

    if(result != CL_SUCCESS){
        qWarning() << "CL object create failed:" << oclErrorString(result);
    }

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
        qWarning() << "Could not link shader program:" << shader.log();

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
    int result = clEnqueueAcquireGLObjects(cmd_queue, 1, &p_vbocl, 0,0,0);
    if(result != CL_SUCCESS){
        qWarning() << "Aquire failed:" << oclErrorString(result);
    }

    // Set queue the kernel
    clSetKernelArg(kernel, 1, sizeof(float), (void*)&anim);
    const size_t buffsize = app_data->cloud->points.size();

    result = clEnqueueNDRangeKernel(cmd_queue, kernel, 1, NULL, &buffsize, NULL, 0, 0, 0);
    if(result != CL_SUCCESS)
        qWarning() << "Kernel exectution failed.";

    // queue unmap buffer object
    result = clEnqueueReleaseGLObjects(cmd_queue, 1, &p_vbocl, 0,0,0);
    if(result != CL_SUCCESS){
        qWarning() << "Release failed:" << oclErrorString(result);
    }

    result = clFinish(cmd_queue);
    if(result != CL_SUCCESS)
        qWarning() << "Finish failed";

    // Clear the buffer with the current clearing color
    glClearDepth(1.0f);
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Calculate modelview matrix
    glm::mat4 translate(1.0f);
    translate[3]+= offsetVec;
    modelview_mat = viewPole->CalcMatrix() * translate * objtPole->CalcMatrix();

    point_shader.bind();
    glUniformMatrix4fv(point_shader.uniformLocation("modelToCameraMatrix"), 1, GL_FALSE, glm::value_ptr(modelview_mat));
    point_buffer.bind();
    point_shader.enableAttributeArray( "vertex" );
    point_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
    glDrawArrays( GL_POINTS, 0, app_data->cloud->size());
    point_buffer.release();
    point_shader.release();

    // TODO Manual painting
    // TODO Figure out indexed painting

    //float lasso_data[12] = {0.0f, 0.0f, width(), height(), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; //, 0.0f, 1.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 1.0f};

    float lasso_data[4];

    // Set up shader
    lasso_shader.bind();

    //printf("1: %s\n", gluErrorString(glGetError()));

    //glm::mat4 ortho_mat = glm::ortho(0.0f, (float)this->width(), 0.0f, (float)this->height());
    //glUniformMatrix4fv(lasso_shader.uniformLocation("ortho"), 1, GL_FALSE, glm::value_ptr(ortho_mat));


    Eigen::Matrix4f ortho;

    ortho << 2.0f/(float)this->width(), 0, 0, -1,//-width()/2.0f,
             0, -2.0f/(float)this->height(), 0, 1,//-height()/2.0f,
             0, 0, 1, 0,
             0, 0, 0, 1;

    glUniformMatrix4fv(lasso_shader.uniformLocation("ortho"), 1, GL_FALSE, ortho.data());

    lasso_buffer.bind();
    lasso_buffer.write(0, reinterpret_cast<const void *> (lasso_data), sizeof(lasso_data));

    lasso_shader.enableAttributeArray( "point" );
    lasso_shader.setAttributeBuffer( "point", GL_FLOAT, 0, 2 );

    //glDrawArrays( GL_LINES, 0, 6); // count is number of vertices


    if(lasso.size() > 1){
        for(int i = 0; i < lasso.size()-1; i++){
            lasso_data[0] = lasso[i].x();
            lasso_data[1] = lasso[i].y();
            lasso_data[2] = lasso[i+1].x();
            lasso_data[3] = lasso[i+1].y();
            lasso_buffer.write(0, reinterpret_cast<const void *> (lasso_data), sizeof(lasso_data));
            glDrawArrays( GL_LINES, 0, 4);
        }

        lasso_data[0] = lasso[0].x();
        lasso_data[1] = lasso[0].y();
        lasso_buffer.write(0, reinterpret_cast<const void *> (lasso_data), sizeof(lasso_data));
        glDrawArrays( GL_LINES, 0, 4);
    }

    lasso_buffer.release();
    lasso_shader.release();

    //printf("5: %s\n", gluErrorString(glGetError()));
}

void GLWidget::addLassoPoint(int x, int y){
    if(!lasso_active)
        return;

    if(lasso.empty()){
        lasso.push_back(Eigen::Vector2i(x,y));
        lasso.push_back(Eigen::Vector2i(x,y));
    }
    else{
        Eigen::Vector2i end = lasso.back();
        lasso.back() = Eigen::Vector2i(x, y);
        lasso.push_back(end);
    }
}

void GLWidget::moveLasso(int x, int y){
    if(!lasso.empty() && lasso_active){
        lasso.back() = Eigen::Vector2i(x, y);
    }
}


void GLWidget::mouseDoubleClickEvent ( QMouseEvent * event ){
    // End lasso
    lasso_active = !lasso_active;
    if(lasso_active)
        addLassoPoint(event->x(), event->y());
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
    }

    updateGL();
}

void GLWidget::mouseReleaseEvent ( QMouseEvent * event ){

    objtPole->MouseClick(glutil::MB_LEFT_BTN, false, 0, glm::ivec2(event->x(), event->y()));
    viewPole->MouseClick(glutil::MB_RIGHT_BTN, false, 0, glm::ivec2(event->x(), event->y()));

    if(!moved){
        printf("CLICK CLICK!!\n");
        clickity(event->x(), event->y());
    }

    if (!moved && event->button() == Qt::LeftButton){
            addLassoPoint(event->x(), event->y());
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

    // Points
    printf("Point 1: (%f, %f, %f)\n", p1.x, p1.y, p1.z);
    printf("Point 2: (%f, %f, %f)\n", p2.x, p2.y, p2.z);

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

    // point filling
    float empty2[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    point_buffer.write(4*sizeof(float)*min_index, reinterpret_cast<const void *> (empty2), sizeof(empty2));

    // 3d brush
    if(filling){
        printf("filling!!\n");
        std::vector<bool> filled(app_data->cloud->points.size(), false);
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

    case Qt::Key_F:
            filling = !filling;
            printf("filling toggle\n");
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
        int offset = 4*sizeof(float)*i;
        point_buffer.write(offset, reinterpret_cast<const void *> (data), sizeof(data));
    }
    std::printf("Reloaded\n");
 }
