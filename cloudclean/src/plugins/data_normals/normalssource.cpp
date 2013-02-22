#include "normalssource.h"
#include <QDebug>

NormalsSource::NormalsSource(){}
NormalsSource::~NormalsSource(){}
void NormalsSource::flushCache(){
    qDebug() << "Woot woot! Fluss all!";
}

void NormalsSource::flushCloud(std::shared_ptr<PointCloud> cloud){
    qDebug() << "Woot woot! Fluss cloud!";
}

NormalsSource * NormalsSourceFactory:: getInstance(){
    return new NormalsSource();
}


Q_EXPORT_PLUGIN2(pnp_data, NormalsSourceFactory)
