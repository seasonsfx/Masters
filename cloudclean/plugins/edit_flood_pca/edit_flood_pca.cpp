#define GL3_PROTOTYPES
#include <gl3.h>
#include <GL/glu.h>

#include <QIcon>
#include <QDebug>
#include <QResource>
#include <QAbstractItemView>
#include <QTime>

#include <omp.h>
#include <pcl/common/pca.h>

#include "edit_flood_pca.h"
#include "utilities.h"
#include "layer.h"
#include "glarea.h"
#include "cloudmodel.h"


EditPlugin::EditPlugin()
{
    settings = new Settings();
    kPCA = settings->kPCA();
    editSample = new QAction(QIcon(":/images/brush.png"), "Floodfill (PCA)", this);
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

void EditPlugin::calcPCA(CloudModel *cm){

    kPCA = settings->kPCA();

    QTime t;
    t.start();

    // For every value
    for(int i = 0; i < cm->cloud->size(); i++){

        boost::shared_ptr <std::vector<int> > kIdxs;
        kIdxs = boost::shared_ptr <std::vector<int> >(new std::vector<int>);
        vector<float> kDist;
        octree->nearestKSearch(i, kPCA , *kIdxs, kDist);

        /*for(int n: *kIdxs.get()){
            qDebug("Idx: %d", n);
        }*/

        //std::vector<int> t = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9 ,10, 11, 12, 13, 14, 15};
        //boost::shared_ptr<vector<int> > idxs(new vector<int>(t));
        //pcEstimator.setIndices(idxs);

        pcl::PCA<pcl::PointXYZI> pcEstimator;
        pcEstimator.setInputCloud (cm->cloud);
        pcEstimator.setIndices(kIdxs);
        eigen_vals->at(i) = pcEstimator.getEigenValues();

        // Sort and normalise
        float * eigenvalues = eigen_vals->at(i).data();

        for(int j = 0; j < 3; j++ ){
            int max = j;
            for(int k = j; k < 3; k++ ){
                if(eigenvalues[k] > eigenvalues[max])
                    max = k;
            }
            float tmp = eigenvalues[j];
            eigenvalues[j] = eigenvalues[max];
            eigenvalues[max] = tmp;
        }

        eigen_vals->at(i).normalize();
        //qDebug("Eigen values: %f %f %f", eigen_vals->at(i).x(), eigen_vals->at(i).y(), eigen_vals->at(i).z());
    }

    qDebug("Time to calc PCA: %d ms", t.elapsed());

}

bool EditPlugin::StartEdit(QAction *action, CloudModel *cm, GLArea *glarea){
    // Set up kdtree and octre if not set up yet

    if(octree.get() == NULL){
        QTime t;
        t.start();

        double resolution = 0.2;

        octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(resolution));
        octree->setInputCloud(cm->cloud);
        octree->defineBoundingBox();
        octree->addPointsFromInputCloud();

        qDebug("Time to create octree: %d ms", t.elapsed());

        eigen_vals = boost::shared_ptr<std::vector<Eigen::Vector3f> >(new std::vector<Eigen::Vector3f>());
        eigen_vals->resize(cm->cloud->size());

        calcPCA(cm);
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

    QTime t;
    t.start();

    int min_index = glarea->pp->pick(x, y, source_idx);

    qDebug("Time to pick point: %d ms", t.elapsed());

    if(min_index == -1)
        return;

    t.start();
    // Recalculate PCA if the K neighbourhood has changed
    if(kPCA != settings->kPCA())
        calcPCA(cm);

    std::queue<int> myqueue;
    myqueue.push(min_index);
    int current;

    std::vector<int> & source = cm->layerList.layers[source_idx].index;
    std::vector<int> & dest = cm->layerList.layers[dest_idx].index;

    int K = settings->kConnectvity();
    float threshold = settings->deviation();

    Eigen::Vector3f ratio = settings->ratio();

    while (!myqueue.empty()){
        current = myqueue.front(); myqueue.pop();

        // Skip invalid indices, visited indices are invalid
        if(source[current] == -1)
            continue;

        // copy from source to dest
        dest[current] = source[current];
        source[current] = -1;

        // push neighbours..

        int idx = -1;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        octree->nearestKSearch (cm->cloud->points[current], K, pointIdxNKNSearch, pointNKNSquaredDistance);

        for (int i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
            idx = pointIdxNKNSearch[i];

            // Skip invalid indices, visited indices are invalid
            if(source[idx] == -1)
                continue;

            Eigen::Vector3f & neighbour = eigen_vals->at(idx);

            // Calculate
            float similarity = angularSimilarity(neighbour, ratio);

            /*
            qDebug("Ratio: %f %f %f, eigen: %f %f %f",
                   ratio.x(), ratio.y(), ratio.z(),
                   current.x(), current.y(), current.z());

            qDebug("Similarity %f", similarity);
*/
            if(similarity > threshold)
                continue;

            myqueue.push(idx);
        }
    }

    qDebug("Time to fill : %d ms", t.elapsed());
    t.start();

    cm->layerList.layers[source_idx].cpu_dirty = true;
    cm->layerList.layers[dest_idx].cpu_dirty = true;

    cm->layerList.layers[source_idx].sync();
    cm->layerList.layers[dest_idx].sync();

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
                l.sync();;
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
