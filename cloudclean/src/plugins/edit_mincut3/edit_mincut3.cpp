#include <QIcon>
#include <QDebug>
#include <QResource>
#include <QAbstractItemView>
#include <QTime>

//#include <pcl/segmentation/min_cut_segmentation.h>

#include "edit_mincut3.h"
#include "utilities.h"
#include "layer.h"
#include "glarea.h"
#include "cloudmodel.h"
#include "mincut.h"


EditPlugin::EditPlugin()
{
    settings = new Settings();
    editSample = new QAction(QIcon(":/images/mincut.svg"), "Min cut 3", this);
    actionList << editSample;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

EditPlugin::~EditPlugin()
{
    delete settings;
}

void EditPlugin::paintGL(CloudModel *, GLArea * glarea){

}

bool EditPlugin::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    return true;
}

bool EditPlugin::StartEdit(QAction *action, CloudModel *cm, GLArea *glarea){
    // Set up kdtree and octre if not set up yet

    cm->layerList.setSelectMode(QAbstractItemView::SingleSelection);

    dest_layer = -1;
    return true;
}
bool EditPlugin::EndEdit(CloudModel * cm, GLArea *){
    emit cm->layerList.setSelectMode(QAbstractItemView::MultiSelection);

    for(auto a: actionList){
        a->setChecked(false);
    }
    return true;
}

inline float clamp(float x, float a, float b)
{
    return x < a ? a : (x > b ? b : x);
}

inline float angularSimilarity(Eigen::Vector3f &a, Eigen::Vector3f &b){
    float cosine = a.dot(b) / (a.norm()*b.norm());
    cosine = clamp(cosine, 0.0f, 1.0f);
    float angularSimlarity = acos(cosine)/M_PI;
    return angularSimlarity;
}

void EditPlugin::fill(int x, int y, float radius, int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea){

    int index = glarea->pp->pick(x, y, source_idx);

    if(index == -1)
        return;

    pcl::PointXYZI p = cm->cloud->points[index];

    // Need to get rid of the -1 from the indices
    pcl::IndicesPtr source_indices(new std::vector<int>);
    for(int idx : cm->layerList.layers[source_idx].index){
        if(idx == -1)
            continue;
        source_indices->push_back(idx);
    }

    MinCut seg;
    seg.setInputCloud(cm->cloud);
    seg.setIndices(pcl::IndicesPtr(new std::vector<int>(cm->layerList.layers[source_idx].index)));
    seg.setNormals(cm->normals);

    pcl::PointCloud<pcl::PointXYZI>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZI> ());
    foreground_points->points.push_back(p);
    seg.setForegroundPoints (foreground_points);
    seg.setRadius (settings->radius());

    seg.setSigma (settings->sigma()); // I think this is density

    seg.setNumberOfNeighbours (settings->kConnectvity());
    seg.setSourceWeight (settings->sourceWeight());
    //seg.setHorisonalRadius(settings->horisontalRadius());

    std::vector <pcl::PointIndices> clusters;
    seg.extract (clusters);

    assert(clusters.size() != 0);

    // blank source & dest
    for(unsigned int i = 0; i < cm->cloud->points.size(); i++){
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

    }

    return true;
}

bool EditPlugin::mouseReleaseEvent(QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    if (!glarea->moved && event->button() == Qt::LeftButton){
        // Single selection switched on, on edit start

        int source_layer = -1;

        for(unsigned int i = 0; i < cm->layerList.layers.size(); i++){
            Layer & l = cm->layerList.layers[i];
            if(l.active && l.visible){
                source_layer = i;
                l.sync();;
                break;
            }
        }


        if(dest_layer == -1 || (unsigned int)dest_layer >= cm->layerList.layers.size()){
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
