#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include <GL/glu.h>
#include "edit_brush_fpfh.h"
#include <QIcon>
#include <QDebug>
#include <QResource>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "utilities.h"
#include "layer.h"
#include "QAbstractItemView"
#include <QTime>
#include <pcl/features/fpfh_omp.h>
#include <omp.h>

EditBrush::EditBrush()
{

    editSample = new QAction(QIcon(":/images/brush.png"), "Brush select (FPFH)", this);
    actionList << editSample;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

EditBrush::~EditBrush()
{
}

void EditBrush::paintGL(CloudModel *, GLArea * glarea){

}

bool EditBrush::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    return true;
}

bool EditBrush::StartEdit(QAction *action, CloudModel *cm, GLArea *glarea){
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

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);

        qDebug("Num of CPU: %d", omp_get_num_procs());

        t.start();

        pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud (cm->cloud);
        fpfh.setInputNormals (cm->normals);
        fpfh.setSearchMethod(tree);
        int procs = omp_get_num_procs()/2; // Hyperthreading is bad
        fpfh.setNumberOfThreads(procs);

        fpfhs = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33> ());

        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
        fpfh.setRadiusSearch (0.05);
        //fpfh.setKSearch(5);

        fpfh.compute (*fpfhs);
/*
        for(int i = 0; i < fpfhs->size(); i++){
            pcl::FPFHSignature33 & sig = fpfhs->at(i);
            for(int j = 0; j < 33; j++){
                printf("%f ", sig.histogram[j]);
            }
            printf("\n");
            fflush(stdout);
        }
*/
        qDebug("Time to calc FPFH: %d ms", t.elapsed());

    }

    cm->layerList.setSelectMode(QAbstractItemView::SingleSelection);

    dest_layer = -1;
    return true;
}
bool EditBrush::EndEdit(CloudModel * cm, GLArea *){
    emit cm->layerList.setSelectMode(QAbstractItemView::MultiSelection);

    for(auto a: actionList){
        a->setChecked(false);
    }
    return true;
}

void clickRay(int x, int y, Eigen::Vector3f& p1, Eigen::Vector3f& p2, GLArea * glarea){

    double dX, dY, dZ, dClickY;

    // Convert matrices to doubles
    double mvmatrix[16];
    double projmatrix[16];
    for (int i = 0; i < 16; ++i){
        projmatrix[i] = glarea->camera.projectionMatrix().data()[i];
        mvmatrix[i] = glarea->camera.modelviewMatrix().data()[i];
    }

    // Fetch viewport
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    // Invert y axis
    dClickY = double (glarea->height() - y);

    // Unproject
    gluUnProject ((double) x, dClickY, 0.0, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    p1 = Eigen::Vector3f ( (float) dX, (float) dY, (float) dZ );
    gluUnProject ((double) x, dClickY, 1.0f, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    p2 = Eigen::Vector3f ( (float) dX, (float) dY, (float) dZ );
}

inline float pointToLineDist(Eigen::Vector3f point, Eigen::Vector3f x1, Eigen::Vector3f x2){
    return (x2-x1).cross(x1-point).squaredNorm()/(x2-x1).squaredNorm();
}

int EditBrush::pointPick(int x, int y, float radius, int source_idx, Eigen::Vector3f& p1, Eigen::Vector3f& p2, CloudModel *cm, GLArea * glarea){

    Layer & layer = cm->layerList.layers[source_idx];

    clickRay(x, y, p1, p2, glarea);
    Eigen::Vector3f line = p2-p1;

    int min_index = -1;
    float min_val = FLT_MAX;

    // Need to narrow down candidates

    Eigen::Vector3f& origin = p1;
    Eigen::Vector3f direction = p2-p1;
    std::vector<int> intercept_indices;

    octree->getIntersectedVoxelIndices(origin, direction, intercept_indices);


    // Find point
    for (int i: intercept_indices) {

        //skip indices not in layer
        if(layer.index[i] == -1)
            continue;

        // Save some memory by using map
        pcl::PointXYZI & p = cm->cloud->points[i];
        Eigen::Vector3f point = Eigen::Vector3f(p.x, p.y, p.z);
        //Eigen::Map<Eigen::Vector3f> point = Eigen::Vector3f::Map(p.data, 3);

        // Skip points not on the ray
        Eigen::Vector3f projPoint = (point.dot(line)/line.dot(line))*line;
        float lineLength = sqrt(line.squaredNorm());
        float distViaPoint = sqrt((p2-projPoint).squaredNorm()) + sqrt((projPoint-p1).squaredNorm());
        if(lineLength-distViaPoint > 0.01)
            continue;

        // Skip points to far from the ray
        // Uncomment this to restore functionality
        /*
        float projDist = (projPoint-point).squaredNorm();
        if(projDist > radius)
            continue;
        */

        float dist = pointToLineDist(point, p1, p2);

        if(dist < min_val){
            min_index = i;
            min_val = dist;
        }
    }

    if(min_index == -1)
        qDebug("Cannot find closest point\n");

    return min_index;
}

void EditBrush::fill(int x, int y, float radius, int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea){

    Eigen::Vector3f p1, p2;

    QTime t;
    t.start();

    int min_index = pointPick(x, y, radius, source_idx, p1, p2, cm, glarea);

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

    while (!myqueue.empty() && count++ < 10000){
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

        pcl::FPFHSignature33 & current_sig = fpfhs->at(current);

        for (int i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
            idx = pointIdxNKNSearch[i];

            // Skip invalid indices, visited indices are invalid
            if(source[idx] == -1)
                continue;

            // Skip if to far in feature space
            pcl::FPFHSignature33 & neigbour_sig = fpfhs->at(idx);

            float sum = 0;
            for(int i = 0; i < 33; i++){
                //qDebug("%f - %f", current_sig.histogram[i], neigbour_sig.histogram[i]);
                sum += powf(current_sig.histogram[i] - neigbour_sig.histogram[i], 2);
            }
            float dist = sqrt(sum);

            qDebug("Dist %f", dist);

            if(dist > 40)
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

bool EditBrush::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){

    return true;
}

bool EditBrush::mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * glarea){

    if(glarea->moved){

    }

    return true;
}

bool EditBrush::mouseReleaseEvent(QMouseEvent *event, CloudModel * cm, GLArea * glarea){

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

QList<QAction *> EditBrush::actions() const{
    return actionList;
}
QString EditBrush::getEditToolDescription(QAction *){
    return "Info";
}

Q_EXPORT_PLUGIN2(pnp_editbrush, EditBrush)
