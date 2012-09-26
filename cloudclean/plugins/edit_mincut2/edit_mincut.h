#ifndef EDITPLUGIN_H
#define EDITPLUGIN_H

#include <vector>

#include <QObject>
#include <QAction>
#include <QGLShaderProgram>

#include <Eigen/Dense>

#include <pcl/search/search.h>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include "../../common/interfaces.h"
#include "lasso.h"
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
    void paintGL(CloudModel *, GLArea *glarea);
    bool mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea);
    bool mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea);
    bool mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * glarea);
    bool mouseReleaseEvent(QMouseEvent *event, CloudModel * cm, GLArea * glarea);
    QList<QAction *> actions() const;
    QString getEditToolDescription(QAction *);
    QWidget * getSettingsWidget(QWidget *);

private:
    void segment(int source_idx, int dest_idx, CloudModel *cm, GLArea *glarea);

private:
    QList <QAction *>                       actionList;
    QAction *                               editSample;

    Settings *                              settings;

    int                                     dest_layer;

    Lasso                                   lasso;
    bool                                    lasso_active;
    Eigen::Vector2f                         normalised_mouse_loc;


};

#endif // EDITPLUGIN_H
