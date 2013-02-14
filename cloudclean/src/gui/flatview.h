#ifndef FLATVIEW_H
#define FLATVIEW_H

#include <QWidget>
#include <QImage>
#include <QPainter>
#include "model/cloudlist.h"
#include "model/layerlist.h"
#include "model/pointcloud.h"

class FlatView : public QWidget {
    Q_OBJECT
 public:
    FlatView(std::shared_ptr<CloudList> cl, std::shared_ptr<LayerList> ll);
    virtual void paintEvent(QPaintEvent*);

 private:
    QPoint imageToScanCoord(int x, int y);
    QPoint scanToImageCoord(int x, int y);

 public slots:
    void updateImage();
    void setCloud(std::shared_ptr<PointCloud> pc);
    void syncLabels();
    void syncFlags();

 private:
   QImage img_;
   std::weak_ptr<PointCloud> pc_;
   std::shared_ptr<CloudList> cl_;
   std::shared_ptr<LayerList> ll_;
};

#endif // FLATVIEW_H
