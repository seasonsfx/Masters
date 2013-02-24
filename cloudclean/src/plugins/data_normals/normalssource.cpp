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

QString NormalsSource::getName(){
    return "data_normals";
}

Q_EXPORT_PLUGIN2(data_normals, NormalsSource)
