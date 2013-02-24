#ifndef NORMALSSOURCE_H
#define NORMALSSOURCE_H

#include "plugins/plugininterfaces.h"

class NormalsSource : public DataSourceIFace {
    Q_OBJECT
    Q_INTERFACES(DataSourceIFace)
 public:
    NormalsSource();
    virtual ~NormalsSource();
    QString getName();

 public slots:
    virtual void flushCache();
    virtual void flushCloud(std::shared_ptr<PointCloud> cloud);
};

#endif // NORMALSSOURCE_H
