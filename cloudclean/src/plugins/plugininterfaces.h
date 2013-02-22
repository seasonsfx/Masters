#ifndef DATASOURCEINTERFACE_H
#define DATASOURCEINTERFACE_H

#include <memory>
#include <QtPlugin>
#include "model/pointcloud.h"


class DataSourceIFace : public QObject {
    Q_OBJECT
 public:
    DataSourceIFace();
    virtual ~DataSourceIFace() = 0;

 public slots:
    virtual void flushCache() = 0;
    virtual void flushCloud(std::shared_ptr<PointCloud> cloud) = 0;
};

//Q_DECLARE_INTERFACE(DataSourceIFace, "za.co.circlingthesun.cloudclean.datasourceiface/1.0")

class DataSourceFactoryIFace {
 public:
    virtual DataSourceIFace * getInstance() = 0;
};

Q_DECLARE_INTERFACE(DataSourceFactoryIFace, "za.co.circlingthesun.cloudclean.datasourcefactoryiface/1.0")

#endif // DATASOURCEINTERFACE_H
