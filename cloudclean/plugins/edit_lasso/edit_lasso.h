#ifndef EDITLASSO_H
#define EDITLASSO_H

#include <vector>

#include <QObject>
#include <QAction>

#include <QGLShaderProgram>
#include <QGLBuffer>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include <Eigen/Dense>

#include "../../common/interfaces.h"
#include "settings.h"

class GLArea;

class EditLasso : public QObject, public EditPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(EditPluginInterface)
public:
    EditLasso();
    ~EditLasso();

    bool StartEdit(QAction *action, CloudModel * cm, GLArea * glarea);
    bool EndEdit(CloudModel *, GLArea * glarea);
    void paintGL(CloudModel *, GLArea *glarea);
    bool mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea);
    bool mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea);
    bool mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * glarea);
    bool mouseReleaseEvent(QMouseEvent *event, CloudModel *, GLArea * glarea);
    //bool keyReleaseEvent  (QKeyEvent *, CloudModel *, GLArea *){}
    //bool keyPressEvent    (QKeyEvent *, CloudModel *, GLArea *){}
    //bool wheelEvent(QWheelEvent*, CloudModel *, GLArea * ){}
    QList<QAction *> actions() const;
    QString getEditToolDescription(QAction *);
    QWidget * getSettingsWidget(QWidget *glarea);

private:

    static const int USE_CPU = 0;
    static const int USE_GPU = 1;

    int lassoMethod;

    void lassoToLayer(CloudModel *cm, GLArea *glarea);
    void lassoToLayerGPU(CloudModel *cm, GLArea *glarea);
    void lassoToLayerCPU(CloudModel *cm, GLArea *glarea);
    void addLassoPoint(Eigen::Vector2f point);
    void moveLasso(Eigen::Vector2f point);

    QList <QAction *>               actionList;
    QAction *                       editLassoCPU;
    QAction *                       editLassoGPU;

    QGLShaderProgram                lasso_shader;
    QGLBuffer                       lasso_buffer;
    QGLBuffer                       lasso_index;
    bool                            lasso_active;
    std::vector<Eigen::Vector2f>    lasso;
    cl_program                      program;
    cl_kernel                       kernel;
    size_t                          kernelsize;

    Settings *                      settings;
};

#endif // EDITLASSO_H
