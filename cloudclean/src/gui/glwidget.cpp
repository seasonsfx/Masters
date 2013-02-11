#include "glwidget.h"
#include <QtOpenGL>
#include <QResource>
#include <cmath>
#include <cstdlib>

CloudGLData::CloudGLData(std::shared_ptr<PointCloud> pc) {
    // Assumption: cloud size does not change
    pc_ = pc;
    pc_->pc_mutex->lock();

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    //
    // Point buffer setup
    //
    point_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    point_buffer_->create(); CE();
    point_buffer_->bind(); CE();
    size_t vb_size = sizeof(pcl::PointXYZI)*pc->size();
    point_buffer_->allocate(vb_size); CE();
    glEnableVertexAttribArray(0); CE();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*4, 0); CE();
    glEnableVertexAttribArray(1); CE();
    int offset = sizeof(float)*3;
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float)*4,
                          reinterpret_cast<const void *>(offset)); CE();
    point_buffer_->release(); CE();
    //
    // Label buffer setup
    //
    label_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    label_buffer_->create(); CE();
    label_buffer_->bind(); CE();
    label_buffer_->allocate(pc->size()*sizeof(int16_t)); CE();
    glEnableVertexAttribArray(2); CE(); CE();
    glVertexAttribIPointer(2, 1, GL_SHORT, 0, 0);
    label_buffer_->release(); CE();
    //
    // Set up flag buffer
    //
    flag_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    flag_buffer_->create(); CE();
    flag_buffer_->bind(); CE();
    size_t sb_size = sizeof(uint8_t)*pc->size();
    flag_buffer_->allocate(sb_size); CE();
    glEnableVertexAttribArray(3); CE();
    glVertexAttribIPointer(3, 1, GL_BYTE, 0, 0); CE();
    flag_buffer_->release(); CE();

    glBindVertexArray(0);
    pc_->pc_mutex->unlock();
}

CloudGLData::~CloudGLData() {
    glDeleteVertexArrays(1, &vao_);
}

void CloudGLData::sync(){
    if(!pc_->pc_mutex->try_lock())
        return;

    if(pc_->cloud_dirty_){
        point_buffer_->bind(); CE();
        size_t vb_size = sizeof(pcl::PointXYZI)*pc_->size();
        point_buffer_->allocate(vb_size); CE();
        float * pointbuff =
                static_cast<float *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();
        for(int i = 0; i < pc_->size(); i++) {
            pointbuff[i*4] = (*pc_)[i].x;
            pointbuff[i*4+1] = (*pc_)[i].y;
            pointbuff[i*4+2] = (*pc_)[i].z;
            pointbuff[i*4+3] = (*pc_)[i].intensity;
        }
        glUnmapBuffer(GL_ARRAY_BUFFER);
        point_buffer_->release(); CE();
        pc_->cloud_dirty_ = false;
        qDebug() << "Synced cloud";
    }
    if(pc_->labels_dirty_){
        label_buffer_->bind(); CE();
        label_buffer_->allocate(pc_->size()*sizeof(int16_t)); CE();
        int16_t * layerbuff =
                static_cast<int16_t *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();
        for(int i = 0; i < pc_->labels_.size(); i++){
            layerbuff[i] = pc_->labels_[i];
        }
        glUnmapBuffer(GL_ARRAY_BUFFER);
        label_buffer_->release(); CE();
        pc_->labels_dirty_ = false;
        qDebug() << "Synced labels";
    }
    if(pc_->flags_dirty_){
        flag_buffer_->bind(); CE();
        uint8_t * flag_buffer =
                static_cast<uint8_t *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();
        for(int i = 0; i < pc_->size(); i++) {
            flag_buffer[i] = static_cast<uint8_t>(pc_->flags_[i]);
        }
        glUnmapBuffer(GL_ARRAY_BUFFER);
        pc_->flags_dirty_ = false;
        flag_buffer_->release(); CE();
        qDebug() << "Synced flags";
    }
    pc_->pc_mutex->unlock();
}

void CloudGLData::draw(){
    // Assumptions:
    // - shader is loaded
    // - buffertexure is loaded
    glBindVertexArray(vao_);
    sync();
    glDrawArrays(GL_POINTS, 0, pc_->size()); CE();
    glBindVertexArray(vao_);
}

GLWidget::GLWidget(std::shared_ptr<DataModel> & dm, QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent) {
    setFocusPolicy(Qt::StrongFocus);
    camera_move_unit_ = 0.4;
    point_render_size_ = 4.0f;

    selection_color_[0] = 0.0f;
    selection_color_[1] = 0.0f;
    selection_color_[2] = 1.0f;
    selection_color_[3] = 1.0f;

    this->dm_ = dm;
    setMouseTracking(true);
}

GLWidget::~GLWidget() {
}

QSize GLWidget::minimumSizeHint() const {
    return QSize(500, 500);
}

QSize GLWidget::sizeHint() const {
    return QSize(700, 500);
}

void GLWidget::initializeGL() {
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
    uni_modelview_ = program_.uniformLocation("modelview"); RC(uni_modelview_);
    uni_select_color_ = program_.uniformLocation("select_color"); RC(uni_select_color_);
    program_.release();

    //
    // Set camera
    //
    camera_.setDepthRange(0.1f, 100.0f);
    camera_.setAspect(width() / static_cast<float>(height()));

    //
    // Selection color
    //
    program_.bind(); CE();
    glUniform4fv(uni_select_color_, 1, selection_color_); CE();
    program_.release(); CE();

    //
    // Set up color lookup buffer
    //
    color_lookup_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    color_lookup_buffer_->create(); CE();
    color_lookup_buffer_->bind(); CE();
    // allocate space for one color
    color_lookup_buffer_->allocate(sizeof(float)*4); CE();
    color_lookup_buffer_->release(); CE();

    //
    // Set up textures
    //
    glGenTextures(1, &texture_id_); CE();
    glPointSize(point_render_size_);


    // Set ogl state for each cloud
    /*
    std::map<int, PointCloud>::iterator iter;
    for (iter = dm_->clouds_.begin(); iter != dm_->clouds_.end(); iter++) {
        PointCloud & pc = iter->second;
        int idx = iter->first;
        cloudgldata_[idx].reset(new CloudGLData(&pc));
    }
    */

}

void GLWidget::syncDataModel() {
    // What about layers dirty
    // Is it the same as lookup table dirty?

    // Look for new clouds
    if(dm_->clouds_dirty_){
        std::map<int, std::shared_ptr<PointCloud> >::iterator iter;
        for (iter = dm_->clouds_.begin(); iter != dm_->clouds_.end(); iter++) {
            int idx = iter->first;
            std::shared_ptr<PointCloud> pc = iter->second;
            //if(dm_->)
                cloudgldata_[idx].reset(new CloudGLData(pc));
            //}
        }
        qDebug() << "Clouds synced";
        dm_->clouds_dirty_ = false;
    }

    // Check for label changes
    if(dm_->layer_lookup_table_dirty_){
        color_lookup_buffer_->bind(); CE();
        //assert(dm_->last_label_id_ != -1);
        size_t label_buff_size = (dm_->last_label_id_+1)*sizeof(float)*4;
        color_lookup_buffer_->allocate(label_buff_size); CE();

        float * color_lookup_buffer =
                static_cast<float *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();
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
        qDebug() << "Color lookup buffer synced";
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

    syncDataModel();

    // Draw all clouds
    for(std::pair<const int, std::shared_ptr<CloudGLData> > pair: cloudgldata_) {
        pair.second->draw();
    }

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
