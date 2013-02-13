#include "flatview.h"
#include <QDebug>

FlatView::FlatView(std::shared_ptr<DataModel> dm) {
    dm_ = dm;
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
    for(int i = 0; i < pc_->height * pc_->width; i++){
        pcl::PointXYZI & point = pc_->at(i);
        if(point.intensity != point.intensity)
            continue;
        if(point.intensity > max_i)
            max_i = point.intensity;
    }

    for(int x = 0; x < pc_->width; x++){
        for(int y = 0; y < pc_->height; y++){
            int idx = x*pc_->height + y;

            pcl::PointXYZI & point = pc_->at(idx);
            int intensity;
            if(point.intensity != point.intensity)
                intensity = 0;
            else
                intensity = 255*(point.intensity/max_i);

            QColor col(intensity, intensity, intensity);
            QColor sel(0, 0, 255);

            int label_id = pc_->labels_[idx];
            int layer_id = dm_->layer_lookup_table_[label_id];
            QColor & label_col = dm_->layers_[layer_id].color_;

            // Layer colors
            col.setRed(col.red()/255.0f*label_col.red());
            col.setGreen(col.green()/255.0f*label_col.green());
            col.setBlue(col.blue()/255.0f*label_col.blue());

            // Selection
            float mix = 0.5;
            float mix2 = 1.0 - mix;
            if(uint8_t(pc_->flags_[idx]) & uint8_t(PointFlags::selected)
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

void FlatView::setCloud(int id) {
    qDebug("set 2d cloudview");

    if(pc_.get() != NULL){
        disconnect(pc_->ed_.get(), SIGNAL(flagUpdate()), this,
                   SLOT(syncFlags()));
        disconnect(pc_->ed_.get(), SIGNAL(labelUpdate()), this,
                   SLOT(syncLabels()));
    }
    pc_ = dm_->clouds_[id];
    connect(pc_->ed_.get(), SIGNAL(flagUpdate()), this, SLOT(syncFlags()));
    connect(pc_->ed_.get(), SIGNAL(labelUpdate()), this, SLOT(syncLabels()));

    img_ = QImage(pc_->width, pc_->height, QImage::Format_RGB32);
    updateImage();
    this->resize(pc_->width, pc_->height);
    update();
}

void FlatView::syncLabels(){
    qDebug("sync 2d labels");
    updateImage();
    this->resize(pc_->width, pc_->height);
    update();
}

void FlatView::syncFlags(){
    qDebug("sync 2d flags");
    updateImage();
    this->resize(pc_->width, pc_->height);
    update();
}
