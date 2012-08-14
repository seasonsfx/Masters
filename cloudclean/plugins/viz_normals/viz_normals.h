#ifndef VizNormals_H
#define VizNormals_H

#include <vector>

#include <QObject>
#include <QAction>
#include <QGLShaderProgram>
#include <QGLBuffer>

#include <Eigen/Dense>

#include "../../common/interfaces.h"

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

class GLArea;

class VizNormals : public QObject, public VizPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(VizPluginInterface)
public:
    VizNormals();
    ~VizNormals();

    bool StartViz(QAction *action, CloudModel * cm, GLArea * glarea);
    bool EndViz(CloudModel * cm, GLArea * glarea);
    void paintGL(CloudModel * cm, GLArea *glarea);
    void paintLayer(int, CloudModel *, GLArea *);
    QList<QAction *> actions() const;
    QString getEditToolDescription(QAction *);


private:
    QList <QAction *>                       actionList;
    QAction *                               editSample;
    QGLBuffer                               normal_buffer;
    QGLShaderProgram                        normal_shader;
};

#endif // VizNormals_H
