#ifndef VISUALISE_NORMALS_H
#define VISUALISE_NORMALS_H

#include "pluginsystem/iplugin.h"

#include <Eigen/Dense>

#include "glheaders.h"

class QAction;
class QGLShaderProgram;
class QGLBuffer;
class Core;
class CloudList;
class LayerList;
class GLWidget;
class MainWindow;
class NormalEstimator;

class VisualiseNormals : public IPlugin {
    Q_OBJECT
    Q_INTERFACES(IPlugin)
 public:
    QString getName();
    void initialize(Core * core);
    void initialize2(PluginManager * pm);
    void cleanup();

    void initializeGL();

 private:
    void loadGLBuffers();
    void unloadGLBuffers();

 signals:
    void enabling();

 public slots:
    void enable();
    void disable();
    void paint(Eigen::Affine3f, Eigen::Affine3f);

 private:
    Core * core_;
    CloudList * cl_;
    GLWidget * glwidget_;

    MainWindow * mw_;
    QAction * enable_;

    QGLShaderProgram * program_;
    std::vector<QGLBuffer *> normal_buffers_;

    bool is_enabled_;
    bool buffers_loaded_;
    bool initialized_gl;

    float normal_length_;

    NormalEstimator * ne_;

};

#endif  // VISUALISE_NORMALS_H
