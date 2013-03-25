#include "plugins/brush3d/brush3d.h"
#include <QDebug>

QString Brush3D::getName(){
    return "3D Brush Tool";
}

void Brush3D::initialize(PluginManager * pm, CloudList * cl, LayerList * ll){
    cl_ = cl;
    ll_ = ll;
    initialized_gl = false;
    connect((QObject *)pm, SIGNAL(plugin3dMousePressE(QMouseEvent * event)), this, SLOT(plugin3dMousePressE(QMouseEvent*)));
}

void Brush3D::cleanup(){

}

void Brush3D::initializeGL() {
    initialized_gl = true;
}

void Brush3D::plugin3dPaint(Eigen::Affine3f, Eigen::Affine3f){
    if(!initialized_gl) {
        initializeGL();
    }
}

bool Brush3D::plugin3dMouseMoveE(QMouseEvent * event){

}

bool Brush3D::plugin3dMousePressE(QMouseEvent * event){
    qDebug() << "Hello from plugin";
    return false;
}

bool Brush3D::plugin3dMouseReleaseE(QMouseEvent * event){

}

Q_EXPORT_PLUGIN2(pnp_brush3d, Brush3D)
