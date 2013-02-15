#include "flatview.h"
#include <QDebug>

FlatView::FlatView(std::shared_ptr<CloudList> cl, std::shared_ptr<LayerList> ll) {
    cl_ = cl;
    ll_ = ll;
}

void FlatView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    if(!img_.isNull()){
        p.drawImage(0, 0, img_);
    }
}

inline QPoint FlatView::imageToScanCoord(int x, int y){
    return QPoint(x, img_.height()-1-y);
}

inline QPoint FlatView::scanToImageCoord(int x, int y){
    return QPoint(x, img_.height()-1-y);
}

void FlatView::updateImage(){
    float max_i = 0;

    std::shared_ptr<PointCloud> pc = pc_.lock();

    for(int i = 0; i < pc->height * pc->width; i++){
        pcl::PointXYZI & point = pc->at(i);
        if(point.intensity != point.intensity)
            continue;
        if(point.intensity > max_i)
            max_i = point.intensity;
    }

    for(int x = 0; x < pc->width; x++){
        for(int y = 0; y < pc->height; y++){
            int idx = x*pc->height + y;

            pcl::PointXYZI & point = pc->at(idx);
            int intensity;
            if(point.intensity != point.intensity)
                intensity = 0;
            else
                intensity = 255*(point.intensity/max_i);

            QColor col(intensity, intensity, intensity);
            QColor sel(0, 0, 255);

            int label_id = pc->labels_[idx];
            std::shared_ptr<Layer> layer =
                    ll_->layer_lookup_table_[label_id].lock();
            QColor & label_col = layer->color_;

            // Layer colors
            col.setRed(col.red()/255.0f*label_col.red());
            col.setGreen(col.green()/255.0f*label_col.green());
            col.setBlue(col.blue()/255.0f*label_col.blue());

            // Selection
            float mix = 0.5;
            float mix2 = 1.0 - mix;
            if(uint8_t(pc->flags_[idx]) & uint8_t(PointFlags::selected)
                    && point.intensity == point.intensity){
                col.setRed(col.red()*mix + sel.red()*mix2);
                col.setGreen(col.green()*mix + sel.green()*mix2);
                col.setBlue(col.blue()*mix + sel.blue()*mix2);
            }

            QPoint p = scanToImageCoord(x, y);
            img_.setPixel(p.x(), p.y(), col.rgb());
        }
    }
}

void FlatView::setCloud(std::shared_ptr<PointCloud> new_pc) {
    qDebug("set 2d cloudview");

    if(!pc_.expired()){
        std::shared_ptr<PointCloud> old_pc = pc_.lock();
        disconnect(old_pc->ed_.get(), SIGNAL(flagUpdate()), this,
                   SLOT(syncFlags()));
        disconnect(old_pc->ed_.get(), SIGNAL(labelUpdate()), this,
                   SLOT(syncLabels()));
    }
    pc_ = new_pc; //cl_->clouds_[id];
    std::shared_ptr<PointCloud> pc = pc_.lock();
    connect(pc->ed_.get(), SIGNAL(flagUpdate()), this, SLOT(syncFlags()));
    connect(pc->ed_.get(), SIGNAL(labelUpdate()), this, SLOT(syncLabels()));

    img_ = QImage(pc->width, pc->height, QImage::Format_RGB32);
    updateImage();
    this->resize(pc->width, pc->height);
    update();
}

void FlatView::syncLabels(){
    std::shared_ptr<PointCloud> pc = pc_.lock();
    updateImage();
    this->resize(pc->width, pc->height);
    update();
    qDebug("sync 2d labels");
}

void FlatView::syncFlags(){
    std::shared_ptr<PointCloud> pc = pc_.lock();
    updateImage();
    this->resize(pc->width, pc->height);
    update();
    qDebug("sync 2d flags");
}
