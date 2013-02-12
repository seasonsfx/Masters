#ifndef FLATVIEW_H
#define FLATVIEW_H

#include <QWidget>
#include <QImage>
#include <QPainter>
#include "model/datamodel.h"
#include "model/pointcloud.h"

class FlatView : QWidget {
    Q_OBJECT
 public:
    FlatView(std::shared_ptr<DataModel> dm);
    virtual void paintEvent(QPaintEvent*);

 private:
    QPoint imageToScanCoord(int x, int y);
    QPoint scanToImageCoord(int x, int y);

 public slots:
    void setCloud(int id);

 private:
   QPainter p_;
   QImage img_;
   std::shared_ptr<PointCloud> pc_;
   std::shared_ptr<DataModel> dm_;
};

#endif // FLATVIEW_H
