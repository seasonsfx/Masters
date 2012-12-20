#ifndef EDITBRUSH_H
#define EDITBRUSH_H

#include <vector>

#include "gl_global.h"

#include <QObject>
#include <QAction>
#include <QGLShaderProgram>

#include <Eigen/Dense>

#include <pcl/octree/octree.h>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include "../../common/interfaces.h"
#include "settings.h"

class GLArea;

class EditPlugin : public QObject, public EditPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(EditPluginInterface)
public:
    EditPlugin();
    ~EditPlugin();

    bool StartEdit(QAction *action, CloudModel * cm, GLArea * glarea);
    bool EndEdit(CloudModel * cm, GLArea * glarea);
    void paintGL(CloudModel *, GLArea *glarea);
    bool mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea);
    bool mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea);
    bool mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * glarea);
    bool mouseReleaseEvent(QMouseEvent *event, CloudModel * cm, GLArea * glarea);
    //bool keyReleaseEvent  (QKeyEvent *, CloudModel *, GLArea *){}
    //bool keyPressEvent    (QKeyEvent *, CloudModel *, GLArea *){}
    //bool wheelEvent(QWheelEvent*, CloudModel *, GLArea * ){}
    QList<QAction *> actions() const;
    QString getEditToolDescription(QAction *);
    QWidget * getSettingsWidget(QWidget *);

private:
    void calcLocalNoise(CloudModel *cm);
    void fill(int x, int y, float radius, int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea);
    

private:
    QList <QAction *>                       actionList;
    QAction *                               editSample;

    Settings *                              settings;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr  octree;

    boost::shared_ptr<std::vector<float> > noise_vals;

    int                                      dest_layer;

    int                                      kNoise;

};

#endif // EDITBRUSH_H
