#ifndef NORMALSSOURCE_H
#define NORMALSSOURCE_H

#include "plugins/plugininterfaces.h"

class NormalsSource : public DataSourceIFace {
    Q_OBJECT
 public:
    NormalsSource();
    virtual ~NormalsSource();

 public slots:
    virtual void flushCache();
    virtual void flushCloud(std::shared_ptr<PointCloud> cloud);
};

class NormalsSourceFactory : public QObject, public DataSourceFactoryIFace {
    Q_OBJECT
    Q_INTERFACES(DataSourceFactoryIFace)
    virtual NormalsSource * getInstance();
};

#endif // NORMALSSOURCE_H
