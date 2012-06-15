#include "glwidget.h"
#include "qapplication.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <Eigen/Dense>
#include "helpers.h"
#include <time.h>
#include <stdlib.h>

GLWidget::GLWidget(QWidget* parent )
    : QGLWidget(QGLFormat(QGL::HasOverlay)),
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
    glFormat.setOverlay( true );

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


    srand (time(NULL));
}

void GLWidget::initializeGL()
{
    // Set the clear color to black
    glClearColor( 0.1f, 0.1f, 0.1f, 1.0f );


    // Point shader and buffers
    if ( !prepareShaderProgram(point_shader, "shaders/points.vert", "shaders/points.frag" ) )
        return;
    point_shader.bind();
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
    glError("121");

    point_indices.push_back(QGLBuffer(QGLBuffer::IndexBuffer));
    point_colours.push_back(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    bool created = point_indices[0].create();

    printf("Created : %s\n", created?"true":"false");
    glError("128");
    point_indices[0].setUsagePattern( QGLBuffer::DynamicDraw );
    point_indices[0].bind();
    glError("133");
    point_indices[0].allocate(app_data->cloud->size() * sizeof(int) ); // needs to store indices to be rendered
    printf("size in kb: %f\n", (app_data->cloud->size() * 4)/(1024.0f));
    glError("134");

    point_indices[0].write(0,reinterpret_cast<const void *>(&app_data->p_valid_indices->at(0)), app_data->p_valid_indices->size() * sizeof(int));

    point_buffer.release();
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

    // Load the program source into memory
    std::ifstream file("lasso.cl");
    std::string prog(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));
    file.close();
    const char* source = prog.c_str();
    const size_t kernelsize = prog.length()+1;
    int err;
    program = clCreateProgramWithSource(context, 1, (const char**) &source,
                                 &kernelsize, &err);
    clError("Create program failed: ", err);

    // Build the program executable
    err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (err != CL_SUCCESS) {

        clError("Build failed: ", err);

        size_t len;
        char buffer[8096];

        std::cerr << "Error: Failed to build program executable!" << endl;
        clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG,
                           sizeof(buffer), buffer, &len);
        std::cerr << buffer << endl;
        exit(1);
    }

    // Create the compute kernel in the program
    kernel = clCreateKernel(program, "lasso", &err);
        if (!kernel || err != CL_SUCCESS) {
        std::cerr << "Error: Failed to create compute kernel!" << endl;
        exit(1);
    }

}


void GLWidget::initializeOverlayGL()
{
    printf("Hello!!");
    // Lasso shader and buffers
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
    point_shader.bind();
    glUniformMatrix4fv(point_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, glm::value_ptr(cameraToClipMatrix));
    glError("256");
    glViewport( 0, 0, w, qMax( h, 1 ) );
    //updateGL();
}

void GLWidget::resizeOverlayGL(int w, int h)
{

}

void GLWidget::paintGL(){
    // Clear the buffer with the current clearing color
    glClearDepth(1.0f);
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Calculate modelview matrix
    glm::mat4 translate(1.0f);
    translate[3]+= offsetVec;
    modelview_mat = viewPole->CalcMatrix() * translate * objtPole->CalcMatrix();

    point_shader.bind();
    glUniformMatrix4fv(point_shader.uniformLocation("modelToCameraMatrix"), 1, GL_FALSE, glm::value_ptr(modelview_mat));
    glError("274");


    point_buffer.bind();
    point_shader.enableAttributeArray( "vertex" );
    point_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );

    glPointSize(5);
    for(int i = 0; i < point_indices.size(); i++){
        Eigen::Vector3f colour = point_colours[i];
        glUniform3fv(point_shader.uniformLocation("layerColour"), 1, colour.data());
        glError("285");
        point_indices[i].bind();
        glDrawElements(GL_POINTS, app_data->p_valid_indices->size(), GL_UNSIGNED_INT, 0);
        glError("289");
    }

    point_buffer.release();
    point_shader.release();

}

void GLWidget::paintOverlayGL(){
    // Draw lasso shader
    lasso_shader.bind();
    Eigen::Matrix4f ortho;
    ortho.setIdentity();

    glUniformMatrix4fv(lasso_shader.uniformLocation("ortho"), 1, GL_FALSE, ortho.data());
    glError("300");
    lasso_buffer.bind();
    float lasso_data[4];
    lasso_buffer.write(0, reinterpret_cast<const void *> (lasso_data), sizeof(lasso_data));

    glError("305");

    lasso_shader.enableAttributeArray( "point" );
    lasso_shader.setAttributeBuffer( "point", GL_FLOAT, 0, 2 );
    glError("309");
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
    glError("329");
}


volatile inline float rand_range(float from, float to){
    return (rand()/(float)RAND_MAX) * (to-from) + from;
}

void GLWidget::lassoToLayer(){
    makeCurrent();
    point_shader.bind();
    point_indices.push_back(QGLBuffer(QGLBuffer::IndexBuffer));

    //Eigen::Vector3f colour(((RAND_MAX/2.0f)+(rand()/2.0f))/RAND_MAX , ((RAND_MAX/2.0f)+(rand()/2.0f))/RAND_MAX , ((RAND_MAX/2.0f)+(rand()/2.0f))/RAND_MAX); // Set random lightish colour
    Eigen::Vector3f colour(rand_range(0.2f, 1.0f), rand_range(0.2f, 1.0f), rand_range(0.2f, 1.0f));
    point_colours.push_back(colour);
    point_shader.bind();
    point_indices[point_indices.size()-1].create();
    point_indices[point_indices.size()-1].setUsagePattern( QGLBuffer::DynamicDraw );
    point_indices[point_indices.size()-1].bind();
    point_indices[point_indices.size()-1].allocate(app_data->p_valid_indices->size() * sizeof(int) ); // needs to store indices to be rendered

    // Initialise to invalid indices
    for(unsigned int i = 0; i < app_data->p_valid_indices->size(); i++){
        unsigned int val = -1; // TODO: does invalid indices render?
        point_indices[point_indices.size()-1].write(i*sizeof(int),reinterpret_cast<const void *>(&val), sizeof(int));
    }

    point_indices[point_indices.size()-1].release();
    point_shader.release();

    glFinish();
    int result;

    // Create buffers from OpenGL
    cl_mem cl_points = clCreateFromGLBuffer(context, CL_MEM_READ_WRITE, point_buffer.bufferId(), &result);
    clError("CL 1", result);
    cl_mem cl_sidx = clCreateFromGLBuffer(context, CL_MEM_READ_WRITE, point_indices[0].bufferId(), &result);
    clError("CL 2", result);
    cl_mem cl_didx = clCreateFromGLBuffer(context, CL_MEM_READ_WRITE, point_indices[point_indices.size()-1].bufferId(), &result);
    clError("CL 3", result);

    const cl_mem gl_objects[3] = {cl_points, cl_sidx, cl_didx};

    // Aquire OpenGL buffer objects for writing from OpenCL
    result = clEnqueueAcquireGLObjects(cmd_queue, 3, gl_objects, 0,0,0);
    if(result != CL_SUCCESS){
        qWarning() << "Aquire failed:" << oclErrorString(result);
    }

    // Create OpenCL buffer for lasso and matrix
    int lasso_size = lasso.size();
    float lasso_data[lasso_size*2];
    for(int i = 0; i< lasso_size; i++){
        lasso_data[i*2] = lasso[i].x();
        lasso_data[i*2+1] = lasso[i].y();
    }

    cl_mem cl_lasso = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(lasso_data), &lasso_data, &result);


    clError("CL 5", result);

    glm::mat4 gmat = cameraToClipMatrix * modelview_mat;
    float mat[16];

    for(int i = 0 ; i < 4; i++){
        mat[i*4] = gmat[i].x;
        mat[i*4+1] = gmat[i].y;
        mat[i*4+2] = gmat[i].z;
        mat[i*4+3] = gmat[i].w;
    }

    clError("CL 5.1", result);

    // Set the kernel arguments
    result = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void*)&cl_points);
    clError("CL 5", result);
    result =clSetKernelArg(kernel, 1, sizeof(cl_mem), (void*)&cl_sidx);
    clError("CL 6", result);
    result = clSetKernelArg(kernel, 2, sizeof(cl_mem), (void*)&cl_didx);
    clError("CL 8", result);
    result = clSetKernelArg(kernel, 3, sizeof(cl_mem), (void*)&cl_lasso);
    clError("CL 9", result);
    result = clSetKernelArg(kernel, 4, sizeof(int), (void*)&lasso_size);
    clError("CL 10", result);
    result = clSetKernelArg(kernel, 5, sizeof(float) * 16, glm::value_ptr(gmat));
    clError("CL 11", result);

    // Enqueue the kernel
    const size_t kernel_count = app_data->p_valid_indices->size();
    result = clEnqueueNDRangeKernel(cmd_queue, kernel, 1, NULL, &kernel_count, NULL, 0, 0, 0);
    if(result != CL_SUCCESS)
        qWarning() << "Kernel exectution failed.";


    // Release OpenGL buffer objects
    result = clEnqueueReleaseGLObjects(cmd_queue, 3, gl_objects, 0,0,0);
    if(result != CL_SUCCESS){
        qWarning() << "Release failed:" << oclErrorString(result);
    }
    else
        qWarning() << "SUCCESS!:" << oclErrorString(result);

    // Release lasso
    clReleaseMemObject(cl_lasso);

    result = clFinish(cmd_queue);
    if(result != CL_SUCCESS)
        qWarning() << "OpenCL failed";

    lasso.clear();
    fflush(stdout);
}

// Puts mouse in NDC
inline Eigen::Vector2f GLWidget::normalized_mouse(int x, int y){
    return Eigen::Vector2f(x/(width()/2.0f) - 1.0f, -(y/(height()/2.0f) - 1.0f));
}

void GLWidget::addLassoPoint(int x, int y){

    if(!lasso_active)
        return;

    if(lasso.empty()){
        lasso.push_back(normalized_mouse(x, y));
    }
    else{
        Eigen::Vector2f end = lasso.back();
        lasso.back() = normalized_mouse(x, y);
        lasso.push_back(end);
    }

}

void GLWidget::moveLasso(int x, int y){
    if(!lasso.empty() && lasso_active){
        lasso.back() = normalized_mouse(x, y);
    }
}


void GLWidget::mouseDoubleClickEvent ( QMouseEvent * event ){
    // End lasso
    lasso_active = !lasso_active;
    if(lasso_active){
        addLassoPoint(event->x(), event->y());
    }
    else{
        lasso.pop_back();
        lassoToLayer();
    }
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

    if(mouseDown || lasso.size())
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
        click(event->x(), event->y());
    }

    if (!moved && event->button() == Qt::LeftButton){
            addLassoPoint(event->x(), event->y());
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

void GLWidget::click(int x, int y){

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
    /*if(filling){
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
    }*/

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
