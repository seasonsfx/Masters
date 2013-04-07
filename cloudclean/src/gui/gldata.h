#ifndef GLDATA_H
#define GLDATA_H

#include "glheaders.h"
#include <memory>
#include <mutex>
#include <QObject>
#include <QGLBuffer>
#include "model/pointcloud.h"
#include "gui/cloudgldata.h"
#include "model/cloudlist.h"
#include "model/layerlist.h"
#include "gui/export.h"

class DLLSPEC GLData : public QObject {
    Q_OBJECT
 public:
    GLData(QGLContext * glcontext,
                    CloudList * cl,
                    LayerList *ll,
                    QObject *parent = 0);
    ~GLData();
 signals:
    void update();
    
 public slots:
    void reloadCloud(std::shared_ptr<PointCloud> cloud);
    void reloadColorLookupBuffer();
    void deleteCloud(std::shared_ptr<PointCloud> cloud);
    
 public:
    std::shared_ptr<QGLBuffer> color_lookup_buffer_;
    std::map<std::shared_ptr<PointCloud>, std::shared_ptr<CloudGLData> > cloudgldata_;
    float selection_color_[4];


 private:
    CloudList * cl_;
    LayerList * ll_;
    QGLContext * glcontext_;
    std::mutex * clb_mutex_;
};

#endif // GLDATA_H
