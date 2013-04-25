#include "gldata.h"
#include <QDebug>

GLData::GLData(QGLContext * glcontext, CloudList *cl, LayerList *ll, QObject *parent) : QObject(parent) {
    ll_ = ll;
    cl_ = cl;
    glcontext_ = glcontext;

    selection_color_[0] = 0.0f;
    selection_color_[1] = 0.0f;
    selection_color_[2] = 1.0f;
    selection_color_[3] = 1.0f;

    //
    // Set up color lookup buffer
    //
    glcontext_->makeCurrent();
    color_lookup_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    IF_FAIL("create failed") = color_lookup_buffer_->create(); CE();
    IF_FAIL("bind failed") = color_lookup_buffer_->bind(); CE();
    color_lookup_buffer_->allocate(sizeof(float)*4); CE();
    color_lookup_buffer_->release(); CE();

    CloudList * clp = cl_;
    connect(clp, SIGNAL(deletingCloud(std::shared_ptr<PointCloud>)),
            this, SLOT(deleteCloud(std::shared_ptr<PointCloud>)));

}


void GLData::reloadCloud(std::shared_ptr<PointCloud> cloud){
    glcontext_->makeCurrent();
    cloudgldata_[cloud].reset(new CloudGLData(cloud));
    emit update();
}

void GLData::deleteCloud(std::shared_ptr<PointCloud> cloud){
    cloudgldata_.erase(cloudgldata_.find(cloud));
}

void GLData::reloadColorLookupBuffer(){

    //
    // Resize the buffer, then go through the lookup table and get colours
    // from layers
    //
    glcontext_->makeCurrent();
    IF_FAIL("bind failed") = color_lookup_buffer_->bind(); CE();
    size_t label_buff_size = (ll_->last_label_+1)*sizeof(float)*4;
    color_lookup_buffer_->allocate(label_buff_size); CE();

    float * color_lookup_buffer =
            static_cast<float *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();

    // For every label calculate the color

    auto mix = [this] (const LayerSet & layerset) {
        std::vector<const Layer *> selected;

        for(const Layer * layer : layerset) {
            if(!layer->isVisible())
                return QColor(0, 0, 0, 0);

            for(std::weak_ptr<Layer> l : ll_->selection_){
                std::shared_ptr<Layer> selected_layer = l.lock();
                if(layer == selected_layer.get()){
                    selected.push_back(layer);
                    break;
                }
            }
        }

        if(selected.size() == 0){
            selected.push_back(ll_->default_layer_.get());
        }

        float r = 0, g = 0, b = 0;
        float weight = 1.0/selected.size();

        for(const Layer * l : selected) {
            r += l->color_.red() * weight;
            g += l->color_.green() * weight;
            b += l->color_.blue() * weight;
        }

        // Round up
        r += 0.5; g +=0.5; b +=0.5;
        return QColor(r, g, b);

    };

    for(uint i = 0; i < ll_->last_label_+1; i++) {
        const LayerSet &ll = ll_->getLayersForLabel(i);
        QColor color = mix(ll);
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

GLData::~GLData(){
    //disconnect(cl_, SIGNAL(deletingCloud(std::shared_ptr<PointCloud>)),
    //        this, SLOT(deleteCloud(std::shared_ptr<PointCloud>)));
}
