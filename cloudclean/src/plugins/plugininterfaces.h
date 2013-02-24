#ifndef DATASOURCEINTERFACE_H
#define DATASOURCEINTERFACE_H

#include <memory>
#include <QtPlugin>
#include <QString>
#include "model/pointcloud.h"

class CCPlugin: public QObject  {
    Q_OBJECT
 public:
    virtual QString getName() = 0;
};

class DataSourceIFace : public CCPlugin{
    Q_OBJECT
 public:
    virtual ~DataSourceIFace() = 0;

 public slots:
    virtual void flushCache() = 0;
    virtual void flushCloud(std::shared_ptr<PointCloud> cloud) = 0;
};

Q_DECLARE_INTERFACE(DataSourceIFace, "za.co.circlingthesun.cloudclean.datasourceiface/1.0")

#endif // DATASOURCEINTERFACE_H
