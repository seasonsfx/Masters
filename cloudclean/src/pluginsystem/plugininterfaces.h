#ifndef DATASOURCEINTERFACE_H
#define DATASOURCEINTERFACE_H

#include <memory>
#include <QtPlugin>
#include <QString>
#include "model/pointcloud.h"
#include "pluginsystem/iplugin.h"

class DataSourceIFace : public IPlugin{
    Q_OBJECT
 public:
    virtual ~DataSourceIFace() = 0;

 public slots:
    virtual void flushCache() = 0;
    virtual void flushCloud(std::shared_ptr<PointCloud> cloud) = 0;
};

Q_DECLARE_INTERFACE(DataSourceIFace, "za.co.circlingthesun.cloudclean.datasourceiface/1.0")

#endif // DATASOURCEINTERFACE_H
