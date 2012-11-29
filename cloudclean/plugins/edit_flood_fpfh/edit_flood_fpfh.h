#ifndef EDITBRUSH_H
#define EDITBRUSH_H

#include <vector>

#include <QObject>
#include <QAction>
#include <QGLShaderProgram>
#include <Eigen/Dense>
#include <pcl/octree/octree.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include "../../common/interfaces.h"
#include "settings.h"

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
    QWidget * getSettingsWidget(QWidget *);

private:
    void fill(int x, int y, int source_idx, int dest_idx, CloudModel *cm,
              GLArea *glarea);
    int pointPick(int x, int y, float radius, int source_idx,
                  Eigen::Vector3f& p1, Eigen::Vector3f& p2, CloudModel *cm,
                  GLArea * glarea);

private:
    QList <QAction *>                       actionList;
    QAction *                               editSample;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr  octree;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs;

    int                                     dest_layer;

    Settings *                              settings;

};

#endif // EDITBRUSH_H
