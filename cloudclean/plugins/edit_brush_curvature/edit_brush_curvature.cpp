#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include <GL/glu.h>
#include "edit_brush_curvature.h"
#include <QIcon>
#include <QDebug>
#include <QResource>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "utilities.h"
#include "layer.h"
#include "QAbstractItemView"
#include <QTime>
#include <omp.h>
#include <pcl/common/pca.h>


float CURVATURE = 12;
int NEIGHBOURS = 32;


EditBrush::EditBrush()
{

    editSample = new QAction(QIcon(":/images/brush.png"), "Brush select (Curvature)", this);
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

        t.start();

        /*
        pcl::PrincipalCurvaturesEstimation<pcl::PointXYZI, pcl::Normal, pcl::PrincipalCurvatures> pcEstimator;
        pcEstimator.setInputCloud (cm->cloud);
        pcEstimator.setInputNormals (cm->normals);
        pcEstimator.setSearchMethod(tree);
        pcs = pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr(new pcl::PointCloud<pcl::PrincipalCurvatures>());
        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
        pcEstimator.setRadiusSearch (0.05);
        //fpfh.setKSearch(5)
        pcEstimator.compute (*pcs);
        */


        eigen_vals = boost::shared_ptr<std::vector<Eigen::Vector3f> >(new std::vector<Eigen::Vector3f>());
        eigen_vals->resize(cm->cloud->size());

        pcl::PCA<pcl::PointXYZI> pcEstimator;
        pcEstimator.setInputCloud (cm->cloud);



        // For every value
        for(int i = 0; i < cm->cloud->size(); i++){
            //pcl::PointXYZI & p = cm->cloud->at(i);

            boost::shared_ptr <std::vector<int> > kIdxs;
            kIdxs = boost::shared_ptr <std::vector<int> >(new std::vector<int>);
            vector<float> kDist;
            octree->nearestKSearch(i, NEIGHBOURS, *kIdxs, kDist);

            //std::vector<int> t = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9 ,10, 11, 12, 13, 14, 15};
            //boost::shared_ptr<vector<int> > idxs(new vector<int>(t));

            pcEstimator.setIndices(kIdxs);
            //pcEstimator.setIndices(idxs);
            eigen_vals->at(i) = pcEstimator.getEigenValues();
            //qDebug("Eigen values: %f %f %f", eigen_vals->at(i).x(), eigen_vals->at(i).y(), eigen_vals->at(i).z());
        }


        qDebug("Time to calc PCA: %d ms", t.elapsed());

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

        // push neighbours..

        int idx;

        int K = 5;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        octree->nearestKSearch (cm->cloud->points[current], K, pointIdxNKNSearch, pointNKNSquaredDistance);

        //pcl::PrincipalCurvatures & current_pc = pcs->at(current);

        for (int i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
            idx = pointIdxNKNSearch[i];

            // Skip invalid indices, visited indices are invalid
            if(source[idx] == -1)
                continue;

            // Lets do plane select
            // Need to classify points as a plane, line, or edge
            Eigen::Vector3f & evals = eigen_vals->at(i);


            float eigenvalues[3] = {evals.x(), evals.y(), evals.z()};

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


            // Select the biggest 2 eigen values

            float ratio = fabs(eigenvalues[1]/eigenvalues[0]);
            qDebug("Ratio %f", ratio);

            qDebug("Sorted eigen vals: %f, %f, %f", eigenvalues[0], eigenvalues[1], eigenvalues[2]);

            //bool is_edge = eigenvalues[0] * CURVATURE > eigenvalues[1];

            bool is_plane = eigenvalues[1]/eigenvalues[0] > 0.8 && eigenvalues[2]/eigenvalues[0] < 0.2;

            if(!is_plane)
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
