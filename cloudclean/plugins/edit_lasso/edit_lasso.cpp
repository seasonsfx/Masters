#define GL3_PROTOTYPES

#include <iostream>

#define GL3_PROTOTYPES
#include <../../external/gl3.h>

#include <QIcon>
#include <QDebug>
#include <QResource>
#include <QPolygon>
#include <QPainter>
#include <QTime>

#include "cloudmodel.h"
#include "glarea.h"
#include "edit_lasso.h"
#include "utilities.h"

EditLasso::EditLasso()
{
    kernelsize = 0;
    lassoMethod = USE_CPU;
    lasso_active = false;
    editLassoCPU = new QAction(QIcon(":/images/lasso.png"), "Lasso select (CPU)", this);
    actionList << editLassoCPU;
    editLassoGPU = new QAction(QIcon(":/images/lasso.png"), "Lasso select (OpenCL)", this);
    actionList << editLassoGPU;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

    settings = new Settings();

}

EditLasso::~EditLasso()
{
    delete settings;
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

void EditLasso::lassoToLayer(CloudModel * cm, GLArea * glarea){
    if(!cm->isLoaded()){
        lasso.clear();
        return;
    }

    if(cm->layerList.newLayerMode == cm->layerList.CREATE_NEW_LAYER)
        cm->layerList.newLayer();

    if(lassoMethod == USE_GPU)
        lassoToLayerGPU(cm, glarea);
    else
        lassoToLayerCPU(cm, glarea);
}

void EditLasso::lassoToLayerGPU(CloudModel * cm, GLArea * glarea){
    qDebug("Used GPU");

    std::vector<Layer> & layers = cm->layerList.layers;

    QGLBuffer & source = layers[0].gl_index_buffer;
    QGLBuffer & dest = layers[layers.size()-1].gl_index_buffer;

    cm->layerList.activateLayer(layers.size()-1);

    /// Initialise dest to invalid indices
    for(unsigned int i = 0; i < cm->cloud->size(); i++){
        unsigned int val = -1; // TODO: does invalid indices render?
        dest.write(i*sizeof(int),reinterpret_cast<const void *>(&val), sizeof(int));
    }
    dest.release();

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

    Eigen::Matrix4f gmat = glarea->camera.projectionMatrix().matrix() * glarea->camera.modelviewMatrix().matrix();

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
    result = clSetKernelArg(kernel, 5, sizeof(float) * 16, gmat.data());
    clError("CL 11", result);

    // Enqueue the kernel
    const size_t kernel_count = cm->cloud->size();
    result = clEnqueueNDRangeKernel(glarea->cmd_queue, kernel, 1, NULL, &kernel_count, NULL, 0, 0, 0);
    if(result != CL_SUCCESS)
        qWarning() << "Kernel exectution failed.";


    // Release OpenGL buffer objects
    result = clEnqueueReleaseGLObjects(glarea->cmd_queue, 3, gl_objects, 0,0,0);
    if(result != CL_SUCCESS){
        qWarning() << "Release failed:" << oclErrorString(result);
    }
    else
        qWarning() << "SUCCESS!:" << oclErrorString(result);

    // Release lasso
    clReleaseMemObject(cl_lasso);

    result = clFinish(glarea->cmd_queue);
    if(result != CL_SUCCESS)
        qWarning() << "OpenCL failed";

    lasso.clear();
    fflush(stdout);
}

#include "cpu_lasso.h"

void EditLasso::lassoToLayerCPU(CloudModel *cm, GLArea *glarea){
    std::vector<Layer> & layers = cm->layerList.layers;

    Layer & source = layers[0];
    Layer & dest = layers[layers.size()-1];

    /// Create and read source index from gpu
    vector<int> & dest_indices = dest.index;
    vector<int> & source_indices = source.index;

    // Create lasso
    int lasso_size = lasso.size();
    float2 lasso_data[lasso_size];
    for(int i = 0; i< lasso_size; i++){
        lasso_data[i].x = lasso[i].x();
        lasso_data[i].y = lasso[i].y();
    }

    /// Perform the lasso selection
    Eigen::Matrix4f gmat = glarea->camera.projectionMatrix().matrix() * glarea->camera.modelviewMatrix().matrix();
    float * matdata = gmat.data();

    float depth = settings->getDepth();

    /// for each point
    for(unsigned int i = 0; i < cm->cloud->size(); i++){

        /// get point
        int idx = i;
        pcl::PointXYZI p = cm->cloud->points[idx];
        float point[4] = {p.x, p.y, p.z, p.intensity};

        /// project point
        proj(matdata, point);

        if(point[2] > depth || point[2] < 0.0f)
            continue;

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
    source.cpu_dirty = true;
    dest.cpu_dirty = true;

    source.sync();
    dest.sync();

    lasso.clear();
}


void EditLasso::paintGL(CloudModel *, GLArea * glarea){


    lasso_shader.bind();

    float colour[4] = {0.0f, 0.0f, 1.0f, 0.5f};
    glUniform4fv(lasso_shader.uniformLocation("planeColour"), 1, colour);
    float depth = (settings->getDepth()*2)-1;
    glUniform1f(lasso_shader.uniformLocation("depth"), depth);

    glUniformMatrix4fv(lasso_shader.uniformLocation("modelview"), 1, false, glarea->camera.modelviewMatrix().matrix().data());
    glUniformMatrix4fv(lasso_shader.uniformLocation("projection"), 1, false, glarea->camera.projectionMatrix().matrix().data());

    glError("300");
    lasso_buffer.bind();

    glError("305");

    lasso_shader.enableAttributeArray( "point" );
    lasso_shader.setAttributeBuffer( "point", GL_FLOAT, 0, 2 );
    glError("309");

    float square[8] = {
        -1.0f, -1.0f,
        1.0f, -1.0f,
        1.0f, 1.0f,
        -1.0f, 1.0
        };

    lasso_buffer.write(0, reinterpret_cast<const void *> (square), sizeof(square));
    glPointSize(5);
    glDrawArrays( GL_TRIANGLE_FAN, 0, 4);

    lasso_buffer.release();
    lasso_shader.release();
    glError("329");


    QPainter painter(glarea);
    painter.beginNativePainting();
    painter.setPen(Qt::green);

    QPolygonF polygon;

    // Conversion is a bit of a hack
    for(auto p: lasso){
        float x = (p.x()+1)*(glarea->width()/2.0f);
        float y = (-p.y()+1)*(glarea->height()/2.0f);
        polygon << QPointF(x, y);
    }

    painter.drawPolygon(polygon);

    painter.endNativePainting();

}

bool EditLasso::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    // End lasso
    lasso_active = !lasso_active;
    if(lasso_active){
        addLassoPoint(glarea->normalized_mouse(event->x(), event->y()));
    }
    else if(lasso.size() > 2){
        lasso.pop_back();
        lassoToLayer(cm, glarea);
    }

    glarea->updateGL();

    return true;
}

bool EditLasso::StartEdit(QAction *action, CloudModel *cm, GLArea *glarea){

    if(action == editLassoCPU)
        lassoMethod = USE_CPU;
    else
        lassoMethod = USE_GPU;


    if(lasso_buffer.isCreated()){
        return false;
    }

    // OpenGL
    if (!glarea->prepareShaderProgram(lasso_shader, ":/shaders/lasso.vert", ":/shaders/lasso.frag", ""))
    //if (!glarea->prepareShaderProgram(lasso_shader, "/home/rickert/Masters/cloudclean/plugins/edit_lasso/shaders/lasso.vert", "/home/rickert/Masters/cloudclean/plugins/edit_lasso/shaders/lasso.frag" ))
        return false;

    lasso_shader.bind();
    lasso_buffer.create();
    lasso_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
    if ( !lasso_buffer.bind() )
    {
        qWarning() << "Could not bind vertex buffer to the context";
        return false;
    }
    lasso_buffer.allocate(sizeof(float) * 2 * 4);
    if ( !lasso_shader.bind() )
    {
        qWarning() << "Could not bind shader program to context";
        return false;
    }
    lasso_shader.enableAttributeArray( "point" );
    lasso_shader.setAttributeBuffer( "point", GL_FLOAT, 0, 2 );

    lasso_shader.release();
    lasso_buffer.release();

    if(lassoMethod == USE_GPU){
        // Avoid double initialisation
        if(kernelsize != 0)
            return true;

        // OpenCL
        QResource kernel_resource(":/kernels/lasso.cl");

        QByteArray qsource = qUncompress(kernel_resource.data(),kernel_resource.size());

        const char* source = qsource.constData();

        const size_t kernelsize = qsource.size();

        //qDebug("Source (size %ld): %s", kernelsize, source);

        int err;
        program = clCreateProgramWithSource(glarea->context, 1, &source,
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

    }

    connect(settings, SIGNAL(depthChanged(int)), glarea, SLOT(updateGL()), Qt::UniqueConnection);

    return true;
}
bool EditLasso::EndEdit(CloudModel *, GLArea *){
    for(auto a: actionList){
        a->setChecked(false);
    }
    return true;
}

bool EditLasso::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){

    return true;
}

bool EditLasso::mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * glarea){

    if(glarea->moved){
        moveLasso(glarea->normalized_mouse(event->x(), event->y()));
        if(lasso.size())
            glarea->updateGL();
    }

    return true;
}

bool EditLasso::mouseReleaseEvent(QMouseEvent *event, CloudModel *, GLArea * glarea){
    if (!glarea->moved && event->button() == Qt::LeftButton){
            addLassoPoint(glarea->normalized_mouse(event->x(), event->y()));
    }
    qDebug("CLICK - Lasso size: %d", lasso.size());
    return true;
}

QList<QAction *> EditLasso::actions() const{
    return actionList;
}
QString EditLasso::getEditToolDescription(QAction *){
    return "Info";
}

QWidget * EditLasso::getSettingsWidget(QWidget *){
    return settings;
}

Q_EXPORT_PLUGIN2(pnp_editlasso, EditLasso)
