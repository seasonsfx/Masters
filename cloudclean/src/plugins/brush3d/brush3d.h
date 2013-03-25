#ifndef BRUSH_3D_H
#define BRUSH_3D_H

#include "pluginsystem/iplugin.h"
#include "glheaders.h"

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;

class Brush3D : public IPlugin {
    Q_OBJECT
    Q_INTERFACES(IPlugin)
 public:
    QString getName();
    void initialize(PluginManager * pm, CloudList * cl, LayerList * ll);
    void cleanup();

    void initializeGL();

 public slots:
    void plugin3dPaint(Eigen::Affine3f, Eigen::Affine3f);
    //bool plugin3dDoubleClickE(QMouseEvent * event);
    bool plugin3dMouseMoveE(QMouseEvent * event);
    bool plugin3dMousePressE(QMouseEvent * event);
    bool plugin3dMouseReleaseE(QMouseEvent * event);
    //bool plugin3dWheelE(QWheelEvent * event);
    //bool plugin3dKeyPressE(QKeyEvent * event);

 private:
    CloudList * cl_;
    LayerList * ll_;
    bool initialized_gl;

};

#endif  // BRUSH_3D_H
