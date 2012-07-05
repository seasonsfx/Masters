#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include "edit_lasso.h"
#include <QIcon>
#include <QDebug>
#include <QResource>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "utilities.h"

EditLasso::EditLasso()
{
    lasso_active = false;
    editLasso = new QAction(QIcon(":/images/lasso.png"), "Lasso select", this);
    actionList << editLasso;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

EditLasso::~EditLasso()
{
}


void EditLasso::moveLasso(Eigen::Vector2f point){
    if(!lasso.empty() && lasso_active){
        lasso.back() = point;
    }
}

void EditLasso::addLassoPoint(Eigen::Vector2f point){

    if(!lasso_active)
        return;

    if(lasso.empty()){
        lasso.push_back(point);
    }
    else{
        Eigen::Vector2f end = lasso.back();
        lasso.back() = point;
        lasso.push_back(end);
    }

}

/*
void EditLasso::lassoToLayerCPU(CloudModel * cm){

    cm->layerList.newLayer();

    std::vector<Layer> & layers = cm->layerList.layers;

    QGLBuffer & dest = layers[layers.size()-1].gl_index_buffer;
    dest.create();
    dest.setUsagePattern( QGLBuffer::DynamicDraw );
    dest.bind();
    dest.allocate(cm->cloud->size() * sizeof(int) );
    dest.release();

    /// Create and read source index from gpu
    int * dest_indices = new int[cm->cloud->size()/sizeof(int)];
    int * source_indices = new int[cm->cloud->size()/sizeof(int)];

    QGLBuffer & source = layers[0].gl_index_buffer;
    source.bind(); /// Bind source
    source.read(0, source_indices, source.size());

    // Create lasso
    int lasso_size = lasso.size();
    float2 lasso_data[lasso_size];
    for(int i = 0; i< lasso_size; i++){
        lasso_data[i].x = lasso[i].x();
        lasso_data[i].y = lasso[i].y();
    }

    /// Perform the lasso selection
    glm::mat4 gmat = cameraToClipMatrix * modelview_mat;

    /// for each point
    for(unsigned int i = 0; i < cm->cloud->size(); i++){
        /// get point
        int idx = i;
        pcl::PointXYZI p = cm->cloud->points[idx];
        float point[4] = {p.x, p.y, p.z, p.intensity};

        /// project point
        proj(glm::value_ptr(gmat), point);

        /// make 2d
        float2 vertex = {point[0], point[1]};

        /// do lasso test
        bool in_lasso = pointInsidePolygon(lasso_data, lasso_size, vertex);

        if(in_lasso){
            dest_indices[idx] = source_indices[idx];
            source_indices[idx] = -1;
        }
        else{
            dest_indices[idx] = -1;
        }

    }

    /// Write results gpu
    dest.bind();
    dest.write(0, dest_indices, dest.size());
    source.bind();
    source.write(0, source_indices, source.size());

    delete [] source_indices;
    delete [] dest_indices;

    lasso.clear();
    fflush(stdout);
}
*/

void EditLasso::lassoToLayer(CloudModel * cm, GLArea * glarea){
    if(!cm->isLoaded()){
        lasso.clear();
        return;
    }

    cm->layerList.newLayer();

    std::vector<Layer> & layers = cm->layerList.layers;

    QGLBuffer & dest = layers[layers.size()-1].gl_index_buffer;

    cm->layerList.activateLayer(layers.size()-1);

    dest.create();
    dest.setUsagePattern( QGLBuffer::DynamicDraw );
    dest.bind();
    dest.allocate(cm->cloud->size() * sizeof(int) );

    /// Initialise dest to invalid indices
    for(unsigned int i = 0; i < cm->cloud->size(); i++){
        unsigned int val = -1; // TODO: does invalid indices render?
        dest.write(i*sizeof(int),reinterpret_cast<const void *>(&val), sizeof(int));
    }
    dest.release();

    QGLBuffer & source = layers[0].gl_index_buffer;

    glFinish();
    int result;

    // Create buffers from OpenGL
    cl_mem cl_points = clCreateFromGLBuffer(glarea->context, CL_MEM_READ_WRITE, cm->point_buffer.bufferId(), &result);
    clError("CL 1", result);
    cl_mem cl_sidx = clCreateFromGLBuffer(glarea->context, CL_MEM_READ_WRITE, source.bufferId(), &result);
    clError("CL 2", result);
    cl_mem cl_didx = clCreateFromGLBuffer(glarea->context, CL_MEM_READ_WRITE, dest.bufferId(), &result);
    clError("CL 3", result);

    const cl_mem gl_objects[3] = {cl_points, cl_sidx, cl_didx};

    // Aquire OpenGL buffer objects for writing from OpenCL
    result = clEnqueueAcquireGLObjects(glarea->cmd_queue, 3, gl_objects, 0,0,0);
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

    cl_mem cl_lasso = clCreateBuffer(glarea->context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(lasso_data), &lasso_data, &result);

    clError("CL 4", result);

    glm::mat4 gmat = glarea->cameraToClipMatrix * glarea->modelview_mat;

    clError("CL 4.1", result);

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
    const size_t kernel_count = cm->cloud->size();
    result = clEnqueueNDRangeKernel(glarea->cmd_queue, kernel, 1, NULL, &kernel_count, NULL, 0, 0, 0);
    if(result != CL_SUCCESS)
        qWarning() << "Kernel exectution failed.";


    // Release OpenGL buffer objects
    result = clEnqueueReleaseGLObjects(glarea->cmd_queue, 3, gl_objects, 0,0,0);
    if(result != CL_SUCCESS){
        //qWarning() << "Release failed:" << oclErrorString(result);
    }
    else
        //qWarning() << "SUCCESS!:" << oclErrorString(result);

    // Release lasso
    clReleaseMemObject(cl_lasso);

    result = clFinish(glarea->cmd_queue);
    if(result != CL_SUCCESS)
        qWarning() << "OpenCL failed";

    lasso.clear();
    fflush(stdout);
}


void EditLasso::paintGL(CloudModel *, GLArea * glarea){
    // Draw lasso shader
    glClear(GL_DEPTH_BUFFER_BIT );
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
        for(unsigned int i = 0; i < lasso.size()-1; i++){
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

bool EditLasso::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    // End lasso
    lasso_active = !lasso_active;
    if(lasso_active){
        addLassoPoint(glarea->normalized_mouse(event->x(), event->y()));
    }
    else{
        lasso.pop_back();
        lassoToLayer(cm, glarea);
    }

    glarea->updateGL();

    return true;
}

bool EditLasso::StartEdit(CloudModel * cm, GLArea * glarea){

    qDebug("Call: %d", cm->test());
    qDebug("Call: %d", glarea->test());

    // OpenGL
    if (!glarea->prepareShaderProgram(lasso_shader, ":/shaders/lasso.vert", ":/shaders/lasso.frag" ))
        return false;

    lasso_shader.bind();
    lasso_buffer.create();
    lasso_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
    if ( !lasso_buffer.bind() )
    {
        qWarning() << "Could not bind vertex buffer to the context";
        return false;
    }
    lasso_buffer.allocate(sizeof(float) * 12);
    if ( !lasso_shader.bind() )
    {
        qWarning() << "Could not bind shader program to context";
        return false;
    }
    lasso_shader.enableAttributeArray( "point" );
    lasso_shader.setAttributeBuffer( "point", GL_FLOAT, 0, 2 );


    lasso_shader.release();
    lasso_buffer.release();

    // OpenCL
    QResource kernel_resource(":/kernels/lasso.cl");
    //qDebug("Valid resource: %d", kernel_resource.size());
    //qDebug("Valid compressed: %d", kernel_resource.isCompressed());

    QByteArray qsource = qUncompress(kernel_resource.data(),kernel_resource.size());

    const char* source = qsource.constData();

    //qDebug("PROGRAM: %s", source);

    const size_t kernelsize = qsource.size();
    int err;
    program = clCreateProgramWithSource(glarea->context, 1, (const char**) &source,
                                 &kernelsize, &err);
    clError("Create program failed: ", err);

    // Build the program executable
    err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (err != CL_SUCCESS) {

        clError("Build failed: ", err);

        size_t len;
        char buffer[8096];

        std::cerr << "Error: Failed to build program executable!" << std::endl;
        clGetProgramBuildInfo(program, glarea->device, CL_PROGRAM_BUILD_LOG,
                           sizeof(buffer), buffer, &len);
        std::cerr << buffer << std::endl;
        exit(1);
    }

    // Create the compute kernel in the program
    kernel = clCreateKernel(program, "lasso", &err);
        if (!kernel || err != CL_SUCCESS) {
        std::cerr << "Error: Failed to create compute kernel!" << std::endl;
        exit(1);
    }


    return true;
}
bool EditLasso::EndEdit(CloudModel *, GLArea *){
    return true;
}

bool EditLasso::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){
    //qDebug("Apply edit mode\n");
    if(lasso_active)
        addLassoPoint(glarea->normalized_mouse(event->x(), event->y()));;
    return false;
}

bool EditLasso::mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * glarea){

    if(glarea->moved){
        moveLasso(glarea->normalized_mouse(event->x(), event->y()));
        if(lasso.size())
            glarea->updateGL();
    }

    return false;
}

bool EditLasso::mouseReleaseEvent(QMouseEvent *event, CloudModel *, GLArea * glarea){
    if (!glarea->moved && event->button() == Qt::LeftButton){
            addLassoPoint(glarea->normalized_mouse(event->x(), event->y()));
    }
    return false;
}

QList<QAction *> EditLasso::actions() const{
    return actionList;
}
QString EditLasso::getEditToolDescription(QAction *){
    return "Info";
}

/////// CL KERNEL STUFF

struct float2{
    float x, y;
};

int rand(int value)
{
    const int a = 1103515245;
    const int c = 12345;

    return (a*value) + c;
}

float cross2D(float2 lineA, float2 lineB, float2 other)
{
    float2 dA, dB;
    dA.x = lineA.x - other.x; dA.y = lineA.y - other.y;
    dB.x = lineB.x - other.x; dB.y = lineB.y - other.y;

    return dA.x*dB.y - dA.y*dB.x;
}

int side(float a)
{
    if(a < -1e-6)
        return -1;
    if(a > 1e-6)
        return 1;
    return 0;
}

bool oppositeSides(float2 lineA, float2 lineB, float2 pointC, float2 pointD)
{
    float crossC = cross2D(lineA, lineB, pointC);
    float crossD = cross2D(lineA, lineB, pointD);

    int sideC = side(crossC);
    int sideD = side(crossD);

    return sideC != sideD;
}

float pointDistance(float2 pointA, float2 pointB)
{
    float dx = pointA.x - pointB.x;
    float dy = pointA.y - pointB.y;

    return sqrt(dx*dx + dy*dy);
}

bool pointOnLineSegment(float2 lineA, float2 lineB, float2 pointC)
{
    float lineLength = pointDistance(lineA, lineB);
    float viaPoint = pointDistance(lineA, pointC) + pointDistance(lineB, pointC);

    return fabs(viaPoint - lineLength) < 1e-6;
}

bool intersects(float2 lineA, float2 lineB, float2 lineC, float2 lineD)
{
    if(oppositeSides(lineA, lineB, lineC, lineD) && oppositeSides(lineC, lineD, lineA, lineB))
        return true; /// Lines intersect in obvious manner

    /// Line segments either don't intersect or are parallel
    if(pointOnLineSegment(lineA, lineB, lineC) || pointOnLineSegment(lineA, lineB, lineD) ||
       pointOnLineSegment(lineC, lineD, lineA) || pointOnLineSegment(lineC, lineD, lineB))
        return true;

    return false;
}

float randomAngle(int* lastRandom)
{
    *lastRandom = rand(*lastRandom);
    return 2.0f*M_PI*(*lastRandom % 10000)/10000.0f;
}

float2 randomLineSegment(float2 origin, int* lastRandom)
{
    float angle = randomAngle(lastRandom);
    float2 endPoint;
    endPoint.x = 10000.0f*cos(angle) + origin.x;
    endPoint.y = 10000.0f*sin(angle) + origin.y;
    return endPoint;
}

bool pointInsidePolygon(float2* polygon, int n, float2 point)
{
    int lastRandom = rand(1);

    while(true)
    {
        float2 endPoint = randomLineSegment(point, &lastRandom);

        for(int i = 0; i < n; ++i)
            if(pointOnLineSegment(point, endPoint, polygon[i]))
                continue;

        int hits = 0;

        for(int i = 0; i < n; ++i)
            if(intersects(polygon[i], polygon[(i + 1) % n], point, endPoint))
                ++hits;

        return (hits % 2 == 1);
    }
}

//// END CL KERNEL STUFF

Q_EXPORT_PLUGIN2(pnp_editlasso, EditLasso)
