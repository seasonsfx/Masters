#include "flatview.h"

FlatView::FlatView(std::shared_ptr<DataModel> dm): p_(this) {
    dm_ = dm;
}

void FlatView::paintEvent(QPaintEvent*) {
    p_.drawImage(0, 0, img_);
}

inline QPoint FlatView::imageToScanCoord(int x, int y){
    return QPoint(img_.height()-1+y, x);
}

inline QPoint FlatView::scanToImageCoord(int x, int y){
    return QPoint(pc_->height-1+y, x);
}

void FlatView::setCloud(int id){
    // reset image
    // reset size
    // if -1 then reset
    pc_ = dm_->clouds_[id];
    img_ = QImage(pc_->width, pc_->height, QImage::Format_RGB32);

    float max_i = 0;
    for(int i = 0; i < pc_->height * pc_->width; i++){
        pcl::PointXYZI & point = pc_->at(i);
        if(point.intensity != point.intensity)
            continue;
        if(point.intensity > max_i)
            max_i = point.intensity;
    }

    for(int w = 0; w < pc_->width; w++){
        for(int h = 0; h < pc_->height; h++){
            int idx = w*pc_->height + h;
            pcl::PointXYZI & point = pc_->at(idx);
            int intensity;
            if(point.intensity != point.intensity)
                intensity = 255*(point.intensity/max_i);
            else
                intensity = 0;
            img_.setPixel(w, pc_->height-h-1, intensity);
        }
    }

    //

}
