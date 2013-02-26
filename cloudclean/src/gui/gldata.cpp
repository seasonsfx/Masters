#include "gldata.h"
#include <QDebug>

GLData::GLData(std::shared_ptr<CloudList> &cl, std::shared_ptr<LayerList> &ll, QObject *parent) : QObject(parent) {
    ll_ = ll;
    cl_ = cl;

    //
    // Set up color lookup buffer
    //
    qDebug() << "Valid context: " << QGLContext::currentContext()->isValid();
    color_lookup_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    IF_FAIL("create failed") = color_lookup_buffer_->create(); CE();
    IF_FAIL("bind failed") = color_lookup_buffer_->bind(); CE();
    color_lookup_buffer_->allocate(sizeof(float)*4); CE();
    color_lookup_buffer_->release(); CE();
    qDebug() << "created!!!!!!!!!!!!!!!!!!!!";
    qDebug() << this;
}


void GLData::reloadCloud(std::shared_ptr<PointCloud> cloud){
    cloudgldata_[cloud].reset(new CloudGLData(cloud));
    emit update();
    qDebug() << "Cloud "; // << cloud << "loaded";
}

void GLData::reloadColorLookupBuffer(){
    //
    // Resize the buffer, then go through the lookup table and get colours
    // from layers
    //
    qDebug() << "Valid context: " << QGLContext::currentContext()->isValid();
    qDebug() << "created: " << color_lookup_buffer_->isCreated();
    qDebug() << "size: " << color_lookup_buffer_->size();
    qDebug() << "buffid:" << color_lookup_buffer_->bufferId();
    qDebug() << this;
    IF_FAIL("bind failed") = color_lookup_buffer_->bind(); CE();
    size_t label_buff_size = (ll_->last_label_id_+1)*sizeof(float)*4;
    color_lookup_buffer_->allocate(label_buff_size); CE();

    float * color_lookup_buffer =
            static_cast<float *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();

    for(int i = 0; i <= ll_->last_label_id_; i++) {
        bool exists = ll_->layer_lookup_table_.find(i)
                != ll_->layer_lookup_table_.end();
        QColor color;
        if(exists) {
            std::shared_ptr<Layer> layer = ll_->layer_lookup_table_[i].lock();
            color = layer->color_;
        }
        else {
            qWarning() << "Warning! No label associated with this layer";
            color = QColor(255, 0, 0, 255);
        }
        color_lookup_buffer[i*4] = color.red()/255.0f;
        color_lookup_buffer[i*4+1] = color.green()/255.0f;
        color_lookup_buffer[i*4+2] = color.blue()/255.0f;
        color_lookup_buffer[i*4+3] = color.alpha()/255.0f;
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    color_lookup_buffer_->release(); CE();
    qDebug() << "Color lookup buffer synced";
    emit update();
}
