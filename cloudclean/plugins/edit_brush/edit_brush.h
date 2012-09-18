#ifndef EDITBRUSH_H
#define EDITBRUSH_H

#include <QObject>
#include <QAction>
#include "../../common/interfaces.h"
#include <Eigen/Dense>
#include <vector>
#include <QGLShaderProgram>
#include <pcl/octree/octree.h>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

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
    QList<QAction *> actions() const;
    QString getEditToolDescription(QAction *);

private:
    void fill(int x, int y, float radius, int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea);
    

private:
    QList <QAction *>                       actionList;
    QAction *                               editSample;

    //pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr   kdtree;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr  octree;

    int dest_layer;

};

#endif // EDITBRUSH_H
