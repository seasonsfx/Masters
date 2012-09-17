#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include <GL/glu.h>

#include <QIcon>
#include <QDebug>
#include <QResource>
#include <QAbstractItemView>
#include <QTime>

#include "edit_mincut.h"
#include "utilities.h"
#include "layer.h"
#include "glarea.h"
#include "cloudmodel.h"
#include "mincut.h"


EditPlugin::EditPlugin()
{
    normalised_mouse_loc = Eigen::Vector2f(0,0);
    settings = new Settings();
    editSample = new QAction(QIcon(":/images/mincut.svg"), "Min cut 2", this);
    actionList << editSample;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

EditPlugin::~EditPlugin()
{
    delete settings;
}

void EditPlugin::paintGL(CloudModel *, GLArea * glarea){
    lasso.drawLasso(normalised_mouse_loc, glarea);
}

bool EditPlugin::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){
    polygon_lasso = lasso.getPolygon();
    return true;
}

bool EditPlugin::StartEdit(QAction *action, CloudModel *cm, GLArea *glarea){
    cm->layerList.setSelectMode(QAbstractItemView::SingleSelection);
    dest_layer = -1;
    return true;
}
bool EditPlugin::EndEdit(CloudModel * cm, GLArea *){
    emit cm->layerList.setSelectMode(QAbstractItemView::MultiSelection);

    for(auto a: actionList){
        a->setChecked(false);
    }
    polygon_lasso.clear();
    return true;
}

void EditPlugin::fill(int x, int y, float radius, int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea){

    int index = glarea->pp->pick(x, y, source_idx);

    if(index == -1)
        return;

    pcl::PointXYZI p = cm->cloud->points[index];

    // Need to get rid of the -1 from the indices
    pcl::IndicesPtr source_indices(new std::vector<int>);
    for(int idx : cm->layerList.layers[source_idx].index){
        if(idx = -1)
            continue;
        source_indices->push_back(idx);
    }

    MinCut seg;
    seg.setInputCloud(cm->cloud);
    seg.setIndices(pcl::IndicesPtr(new std::vector<int>(cm->layerList.layers[source_idx].index)));

    pcl::PointCloud<pcl::PointXYZI>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZI> ());
    foreground_points->points.push_back(p);
    seg.setForegroundPoints (foreground_points);

    seg.setSigma (settings->sigma());
    seg.setRadius (settings->radius());
    seg.setNumberOfNeighbours (settings->kConnectvity());
    seg.setSourceWeight (settings->sourceWeight());

    std::vector <pcl::PointIndices> clusters;
    seg.extract (clusters);

    assert(clusters.size() != 0);

    // blank source & dest
    for(int i = 0; i < cm->cloud->points.size(); i++){
        cm->layerList.layers[source_idx].index[i] = -1;
        cm->layerList.layers[dest_idx].index[i] = -1;
    }


    // put clusters into layer
    for(int idx : clusters[0].indices){
        //qDebug("Cluster 0:  %d", idx);
        cm->layerList.layers[source_idx].index[idx] = idx;
    }

    for(int idx : clusters[1].indices){
        //qDebug("Cluster 1:  %d", idx);
        cm->layerList.layers[dest_idx].index[idx] = idx;
    }

    cm->layerList.layers[source_idx].copyToGPU();
    cm->layerList.layers[dest_idx].copyToGPU();

}

bool EditPlugin::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){

    return true;
}

bool EditPlugin::mouseMoveEvent (QMouseEvent *event, CloudModel *, GLArea * glarea){

    if(glarea->moved){
        normalised_mouse_loc = glarea->normalized_mouse(
                    event->x(), event->y());
    }

    return true;
}

bool EditPlugin::mouseReleaseEvent(QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    if (!glarea->moved && event->button() == Qt::LeftButton){
        // Single selection switched on, on edit start

        int source_layer = -1;

        for(int i = 0; i < cm->layerList.layers.size(); i++){
            Layer & l = cm->layerList.layers[i];
            if(l.active && l.visible){
                source_layer = i;
                l.copyFromGPU();
                break;
            }
        }

        if(dest_layer == -1 || dest_layer >= cm->layerList.layers.size()){
            cm->layerList.newLayer();
            dest_layer = cm->layerList.layers.size()-1;
        }

        float radius = 1.0f;

        fill(event->x(), event->y(), radius, source_layer, dest_layer, cm, glarea);

    }

    return true;
}

QList<QAction *> EditPlugin::actions() const{
    return actionList;
}
QString EditPlugin::getEditToolDescription(QAction *){
    return "Info";
}

QWidget * EditPlugin::getSettingsWidget(QWidget *){
    return settings;
}

Q_EXPORT_PLUGIN2(pnp_editbrush, EditPlugin)
