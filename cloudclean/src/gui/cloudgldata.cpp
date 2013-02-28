#include "cloudgldata.h"
#include <QDebug>

CloudGLData::CloudGLData(std::shared_ptr<PointCloud> pc) {
    // Assumption: cloud size does not change
    pc_ = pc;

    dirty_labels_ = true;
    dirty_points_ = true;
    dirty_flags_ = true;
    dirty_grid_ = true;


    //
    // Point buffer setup
    //
    point_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    point_buffer_->create(); CE();
    point_buffer_->bind(); CE();
    size_t vb_size = sizeof(pcl::PointXYZI)*pc->size();
    point_buffer_->allocate(vb_size); CE();
    point_buffer_->release(); CE();
    //
    // Label buffer setup
    //
    label_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    label_buffer_->create(); CE();
    label_buffer_->bind(); CE();
    label_buffer_->allocate(pc->size()*sizeof(int16_t)); CE();
    label_buffer_->release(); CE();
    //
    // Set up flag buffer
    //
    flag_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    flag_buffer_->create(); CE();
    flag_buffer_->bind(); CE();
    size_t sb_size = sizeof(uint8_t)*pc->size();
    flag_buffer_->allocate(sb_size); CE();
    flag_buffer_->release(); CE();

    //
    // Set up grid position buffer
    //
    grid_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    grid_buffer_->create(); CE();
    grid_buffer_->bind(); CE();
    size_t gb_size = sizeof(int)*pc->size();
    grid_buffer_->allocate(gb_size); CE();
    grid_buffer_->release(); CE();

    QMetaObject::invokeMethod(this, "syncCloud");
    QMetaObject::invokeMethod(this, "syncFlags");
    QMetaObject::invokeMethod(this, "syncLabels");

    connect(pc_->ed_.get(), SIGNAL(flagUpdate()), this, SLOT(syncFlags()));
    connect(pc_->ed_.get(), SIGNAL(labelUpdate()), this, SLOT(syncLabels()));
}

CloudGLData::~CloudGLData() {
    disconnect(pc_->ed_.get(), SIGNAL(flagUpdate()), this, SLOT(syncFlags()));
    disconnect(pc_->ed_.get(), SIGNAL(labelUpdate()), this, SLOT(syncLabels()));
}

void CloudGLData::setVAO(GLuint vao){
    glBindVertexArray(vao);

    // Point buffer
    point_buffer_->bind(); CE();
    glEnableVertexAttribArray(0); CE();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*4, 0); CE();
    glEnableVertexAttribArray(1); CE();
    int offset = sizeof(float)*3;
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float)*4,
                          reinterpret_cast<const void *>(offset)); CE();
    point_buffer_->release(); CE();

    // Label buffer
    label_buffer_->bind(); CE();
    glEnableVertexAttribArray(2); CE(); CE();
    glVertexAttribIPointer(2, 1, GL_SHORT, 0, 0); CE();
    label_buffer_->release(); CE();

    // Flag buffer
    flag_buffer_->bind(); CE();
    glEnableVertexAttribArray(3); CE();
    glVertexAttribIPointer(3, 1, GL_BYTE, 0, 0); CE();
    flag_buffer_->release(); CE();

    // Grid pos buffer
    grid_buffer_->bind(); CE();
    glEnableVertexAttribArray(4); CE();
    glVertexAttribIPointer(4, 1, GL_INT, 0, 0); CE();
    grid_buffer_->release(); CE();

    glBindVertexArray(0);
}

void CloudGLData::copyCloud(){
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
    qDebug() << "Synced cloud";
}

void CloudGLData::copyLabels(){
    label_buffer_->bind(); CE();
    label_buffer_->allocate(pc_->size()*sizeof(int16_t)); CE();
    int16_t * layerbuff =
            static_cast<int16_t *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();
    for(int i = 0; i < pc_->labels_.size(); i++){
        layerbuff[i] = pc_->labels_[i];
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    label_buffer_->release(); CE();
    qDebug() << "Synced labels";
}

void CloudGLData::copyFlags(){
    flag_buffer_->bind(); CE();
    uint8_t * flag_buffer =
            static_cast<uint8_t *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();
    for(int i = 0; i < pc_->size(); i++) {
        flag_buffer[i] = static_cast<uint8_t>(pc_->flags_[i]);
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    flag_buffer_->release(); CE();
    qDebug() << "Synced flags";
}

void CloudGLData::copyGrid(){
    grid_buffer_->bind(); CE();
    int * gridbuff =
            static_cast<int *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();
    for(int i = 0; i < pc_->cloud_to_grid_map_.size(); i++){
        gridbuff[i] = pc_->cloud_to_grid_map_[i];
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    grid_buffer_->release(); CE();
    qDebug() << "Synced grid";
}

void CloudGLData::syncCloud(){
    dirty_points_ = true;
}

void CloudGLData::syncLabels(){
    dirty_labels_ = true;
}

void CloudGLData::syncFlags(){
    dirty_flags_ = true;
}

void CloudGLData::draw(GLint vao){
    // Assumptions:
    // - shader is loaded
    // - buffertexure is loaded

    if(dirty_points_){
        copyCloud();
        dirty_points_ = false;
    }
    if(dirty_labels_){
        copyLabels();
        dirty_labels_ = false;
    }
    if(dirty_flags_){
        copyFlags();
        dirty_flags_ = false;
    }
    if(dirty_grid_){
        copyGrid();
        dirty_grid_ = false;
    }

    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, pc_->size()); CE();
    glBindVertexArray(vao);
}
