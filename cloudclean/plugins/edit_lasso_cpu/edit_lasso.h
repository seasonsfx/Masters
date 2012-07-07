#ifndef EDITLASSO_H
#define EDITLASSO_H

#include <QObject>
#include <QAction>
#include "../../common/interfaces.h"
#include <Eigen/Dense>
#include <vector>
#include <QGLShaderProgram>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

class GLArea;

class EditLasso : public QObject, public EditPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(EditPluginInterface)
public:
    EditLasso();
    ~EditLasso();

    bool StartEdit(CloudModel * cm, GLArea * glarea);
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

private:

    void lassoToLayer(CloudModel *cm, GLArea *glarea);
    void lassoToLayerCPU(CloudModel *cm);
    void addLassoPoint(Eigen::Vector2f point);
    void moveLasso(Eigen::Vector2f point);

    QList <QAction *> actionList;
    QAction *editLasso;
    QGLShaderProgram        lasso_shader;
    QGLBuffer               lasso_buffer;
    QGLBuffer               lasso_index;
    bool                    lasso_active;
    std::vector<Eigen::Vector2f>   lasso;
    cl_program              program;
    cl_kernel               kernel;
    size_t                  kernelsize;
};

#endif // EDITLASSO_H
