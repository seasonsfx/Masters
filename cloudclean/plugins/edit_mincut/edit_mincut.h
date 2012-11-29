#ifndef EDITPLUGIN_H
#define EDITPLUGIN_H

#include <vector>

#include <QObject>
#include <QAction>
#include <QGLShaderProgram>
#include <QGLBuffer>

#include <Eigen/Dense>

#include <pcl/search/search.h>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include "../../common/interfaces.h"
#include "settings.h"
#include "mincut.h"

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
    void paintGL(CloudModel * cm, GLArea *glarea);
    bool mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea);
    bool mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea);
    bool mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * glarea);
    bool mouseReleaseEvent(QMouseEvent *event, CloudModel * cm, GLArea * glarea);
    QList<QAction *> actions() const;
    QString getEditToolDescription(QAction *);
    QWidget * getSettingsWidget(QWidget *);

private:
    void calcPCA(CloudModel *cm);
    void fill(int x, int y, float radius, int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea);
    

private:
    QList <QAction *>                       actionList;
    QAction *                               editSample;

    Settings *                              settings;

    int                                     dest_layer;
    MinCut                                  seg;

    // Viz data
    boost::shared_ptr<MinCut::gData>        gdata;
    bool                                    gdata_dirty;
    QGLBuffer                               source_edge_buffer;
    QGLBuffer                               sink_edge_buffer;
    QGLBuffer                               bridge_edge_buffer;

    QGLBuffer                               source_edge_weight_buffer;
    QGLBuffer                               sink_edge_weight_buffer;
    QGLBuffer                               bridge_edge_weight_buffer;

    GLuint                                  textures[3];

    QGLBuffer                               sink_vertex_buffer;
    QGLBuffer                               source_vertex_buffer;
    QGLShaderProgram                        viz_shader;

};

#endif // EDITPLUGIN_H
