#include "glwidget.h"
#include <QtOpenGL>
#include <QResource>
#include <cmath>
#include <cstdlib>

CloudGLData::CloudGLData(PointCloud * pc) {
    pc_ = pc;

    //
    // Point buffer setup
    //
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    point_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    point_buffer_->create(); CE();
    point_buffer_->bind(); CE();
    size_t vb_size = sizeof(pcl::PointXYZI)*pc->size();
    point_buffer_->allocate(vb_size); CE();
    float * pointbuff =
            static_cast<float *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY));
    for(int i = 0; i < pc->size(); i++) {
        pointbuff[i*4] = (*pc)[i].x;
        pointbuff[i*4+1] = (*pc)[i].y;
        pointbuff[i*4+2] = (*pc)[i].z;
        pointbuff[i*4+3] = (*pc)[i].intensity;
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    glEnableVertexAttribArray(0); CE();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*4, 0); CE();
    glEnableVertexAttribArray(1); CE();
    int offset = sizeof(float)*3;
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float)*4, reinterpret_cast<const void *>(offset)); CE();
    point_buffer_->release(); CE();

    //
    // Label buffer setup
    //
    label_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    label_buffer_->create(); CE();
    label_buffer_->bind(); CE();
    label_buffer_->allocate(pc->size()*sizeof(int16_t)); CE();
    int16_t * layerbuff =
            static_cast<int16_t *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY));
    for(int i = 0; i < pc->labels_.size(); i++){
        layerbuff[i] = pc->labels_[i];
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    glEnableVertexAttribArray(2); CE();; CE();
    glVertexAttribIPointer(2, 1, GL_SHORT, 0, 0);
    label_buffer_->release(); CE();
    glBindVertexArray(0);
}

CloudGLData::~CloudGLData() {
    glDeleteVertexArrays(1, &vao_);
}

void CloudGLData::draw(){
    glBindVertexArray(vao_);
    // Assumptions:
    // - shader is loaded
    // - buffertexure is loaded

    // check dirty flag
    // load new data if dirty

    glDrawArrays(GL_POINTS, 0, pc_->size()); CE();
    glBindVertexArray(vao_);
}

GLWidget::GLWidget(std::shared_ptr<DataModel> & dm, QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent) {
    setFocusPolicy(Qt::StrongFocus);
    camera_move_unit_ = 0.4;
    point_render_size_ = 1.0f;
    this->dm_ = dm;
    setMouseTracking(true);
}

GLWidget::~GLWidget() {
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(500, 500);
}

QSize GLWidget::sizeHint() const

{
    return QSize(700, 500);
}

void GLWidget::initializeGL()
{
    glClearColor(0.8, 0.8, 0.8, 1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);
    
    //
    // Load shader program
    //
    bool succ = program_.addShaderFromSourceFile(
                QGLShader::Vertex, ":/points.vert"); CE();
    qWarning() << program_.log();
    if (!succ) qWarning() << "Shader compile log:" << program_.log();
    succ = program_.addShaderFromSourceFile(
                QGLShader::Fragment, ":/points.frag"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_.log();
    succ = program_.link(); CE();
    if (!succ) {
        qWarning() << "Could not link shader program_:" << program_.log();
        qWarning() << "Exiting...";
        abort();
    }

    //
    // Resolve uniforms (Does the program need to be bound for this?)
    //
    program_.bind(); CE();
    uni_sampler_ = program_.uniformLocation("sampler"); RC(uni_sampler_);
    uni_projection_ = program_.uniformLocation("projection"); RC(uni_projection_);
    uni_modelview_ = program_.uniformLocation("modelview"); RC( uni_modelview_);
    program_.release();

    //
    // Set camera
    //
    camera_.setDepthRange(0.1f, 100.0f);
    camera_.setAspect(width() / static_cast<float>(height()));
    program_.bind();
    glUniformMatrix4fv(uni_projection_, 1, GL_FALSE,
                       camera_.projectionMatrix().data());
    program_.release();

    //
    // Set up "Global" color lookup buffer
    //
    color_lookup_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    color_lookup_buffer_->create(); CE();
    color_lookup_buffer_->bind(); CE();
    assert(dm_->last_label_id_ != -1);
    size_t label_buff_size = (dm_->last_label_id_+1)*sizeof(float)*4;
    color_lookup_buffer_->allocate(label_buff_size); CE();
    float * color_lookup_buffer =
            static_cast<float *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY));
    for(int i = 0; i <= dm_->last_label_id_; i++) {
        bool exists = dm_->layer_lookup_table_.find(i)
                != dm_->layer_lookup_table_.end();

        QColor color;
        if(exists) {
            int layer_id = dm_->layer_lookup_table_[i];
            color = dm_->layers_[layer_id].color_;
        }
        else {
            qWarning() << "Warning, no label associated with this layer";
            color = QColor(255, 0, 0, 255);
        }
        color_lookup_buffer[i*4] = color.red()/255.0f;
        color_lookup_buffer[i*4+1] = color.green()/255.0f;
        color_lookup_buffer[i*4+2] = color.blue()/255.0f;
        color_lookup_buffer[i*4+3] = color.alpha()/255.0f;
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    dm_->layer_lookup_table_dirty_ = false;
    color_lookup_buffer_->release(); CE();

    //
    // Set up textures
    //
    glGenTextures(1, &texture_id_); CE();
    glPointSize(5);


    // Set ogl state for each cloud
    std::map<int, PointCloud>::iterator iter;
    for (iter = dm_->clouds_.begin(); iter != dm_->clouds_.end(); iter++) {
        PointCloud & pc = iter->second;
        int idx = iter->first;
        cloudgldata_[idx].reset(new CloudGLData(&pc));
    }

}

void GLWidget::paintGL() {

    // Make sure the labels are updates
    // Make sure nothing has changed

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program_.bind(); CE();

    glUniformMatrix4fv(uni_modelview_, 1, GL_FALSE,
                       camera_.modelviewMatrix().data());CE();
    glUniformMatrix4fv(uni_projection_, 1, GL_FALSE,
                       camera_.projectionMatrix().data());

    glUniform1i(uni_sampler_, 0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, color_lookup_buffer_->bufferId()); CE();

    // TODO(Rickert): Check for new clouds here
    // TODO(Rickert): Cloud position in world space

    // Draw all clouds
    for(std::pair<const int, std::shared_ptr<CloudGLData> > pair: cloudgldata_) {
        pair.second->draw();
    }

    glDrawArrays(GL_POINTS, 0, dm_->clouds_[0].size()); CE();
    glBindTexture(GL_TEXTURE_BUFFER, 0); CE();

    program_.release();
}

void GLWidget::resizeGL(int width, int height) {
    camera_.setAspect(width / static_cast<float>(height));
    glViewport(0, 0, width, qMax(height, 1));
}

void GLWidget::mouseDoubleClickEvent(QMouseEvent * event) {

}

void GLWidget::mouseMoveEvent(QMouseEvent * event) {
    if(event->buttons() & Qt::RightButton)
        camera_.mouseMove(event->x(), event->y());
    if(event->buttons() &  Qt::LeftButton)
        camera_.mouseMove(event->x(), event->y());

    if(event->buttons())
        updateGL();
}

void GLWidget::mousePressEvent(QMouseEvent * event) {
    mouse_drag_start_ = QVector2D(0.0f, 0.0f);

    camera_.mouseDown(event->x(), event->y(), event->button());
}

void GLWidget::mouseReleaseEvent(QMouseEvent * event) {
    if (event->button() == Qt::RightButton)
        camera_.mouseRelease(event->x(), event->y());
    else if (event->button() == Qt::LeftButton)
        camera_.mouseRelease(event->x(), event->y());
    updateGL();
}

void GLWidget::wheelEvent(QWheelEvent * event) {
    camera_.mouseWheel(event->delta());
    updateGL();
}

void GLWidget::keyPressEvent(QKeyEvent * event) {
    switch (event->key()) {
    case Qt::Key_Escape:
            QCoreApplication::instance()->quit();
            break;
    case Qt::Key_D:
    case Qt::Key_Right:
            camera_.adjustPosition(camera_move_unit_, 0, 0);
            break;
    case Qt::Key_A:
    case Qt::Key_Left:
            camera_.adjustPosition(-camera_move_unit_, 0, 0);
            break;
    case Qt::Key_W:
    case Qt::Key_Up:
            camera_.adjustPosition(0, 0, camera_move_unit_);
            break;
    case Qt::Key_S:
    case Qt::Key_Down:
            camera_.adjustPosition(0, 0, -camera_move_unit_);
            break;
    case Qt::Key_Q:
            camera_.adjustPosition(0, +camera_move_unit_, 0);
            break;
    case Qt::Key_E:
            camera_.adjustPosition(0, -camera_move_unit_, 0);
            break;
    case Qt::Key_R:
            if (event->modifiers() == Qt::ControlModifier)
                camera_.setPosition(0, 0, 0);
            break;
    case Qt::Key_Plus:
            if (point_render_size_ < 30)
                point_render_size_++;
            glPointSize(point_render_size_);
            break;
    case Qt::Key_Minus:
            if ( point_render_size_ > 1 )
                point_render_size_--;
            glPointSize(point_render_size_);
            break;
    }
    updateGL();
}

bool GLWidget::eventFilter(QObject *object, QEvent *event) {
    Q_UNUSED(object);
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        keyPressEvent(keyEvent);
        return true;
    }
    return false;
}
