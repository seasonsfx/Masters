#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include "edit_brush.h"
#include <QIcon>
#include <QDebug>
#include <QResource>
#include <QTime>
#include <QAbstractItemView>

#include "utilities.h"
#include "layer.h"
#include "glarea.h"
#include "cloudmodel.h"

EditPlugin::EditPlugin()
{

    editSample = new QAction(QIcon(":/images/brush.png"), "Brush select", this);
    actionList << editSample;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

EditPlugin::~EditPlugin()
{
}

void EditPlugin::paintGL(CloudModel *, GLArea * glarea){

}

bool EditPlugin::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    return true;
}

bool EditPlugin::StartEdit(QAction *action, CloudModel *cm, GLArea *glarea){
    // Set up kdtree and octre if not set up yet

    if(octree.get() == NULL){
        QTime t;
        /*t.start();

        kdtree = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        kdtree->setInputCloud(cm->cloud);
        qDebug("Time to create kdtree: %d ms", t.elapsed());

        t.start();
        */

        double resolution = 0.2;

        octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(resolution));
        octree->setInputCloud(cm->cloud);
        octree->defineBoundingBox();
        octree->addPointsFromInputCloud();

        qDebug("Time to create octtree: %d ms", t.elapsed());
    }

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

void EditPlugin::fill(int x, int y, float radius, int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea){

    QTime t;
    t.start();

    int min_index = glarea->pp->pick(x, y, source_idx);

    qDebug("Time to pick point: %d ms", t.elapsed());


    t.start();
    if(min_index == -1)
        return;

    std::queue<int> myqueue;
    myqueue.push(min_index);
    int current;
    int count = 0; // Stops ape shit

    std::vector<int> & source = cm->layerList.layers[source_idx].index;
    std::vector<int> & dest = cm->layerList.layers[dest_idx].index;

    while (!myqueue.empty() /*&& count++ < 10000*/){
        current = myqueue.front(); myqueue.pop();

        // Skip invalid indices, visited indices are invalid
        if(source[current] == -1)
            continue;

        // copy from source to dest
        dest[current] = source[current];
        source[current] = -1;

        //break; // Remove this to restore functionality

        // push neighbours..

        int idx;

        int K = 5;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        octree->nearestKSearch (cm->cloud->points[current], K, pointIdxNKNSearch, pointNKNSquaredDistance);

        for (int i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
            idx = pointIdxNKNSearch[i];

            // Skip invalid indices, visited indices are invalid
            if(source[idx] == -1)
                continue;

            myqueue.push(idx);
        }
    }

    qDebug("Time to fill : %d ms", t.elapsed());
    t.start();
    cm->layerList.layers[source_idx].copyToGPU();
    cm->layerList.layers[dest_idx].copyToGPU();

   qDebug("Time to copy to GPU: %d ms", t.elapsed());

}

bool EditPlugin::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){

    return true;
}

bool EditPlugin::mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * glarea){

    if(glarea->moved){

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


        if(dest_layer == -1){
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

Q_EXPORT_PLUGIN2(pnp_editbrush, EditPlugin)
