#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include <GL/glu.h>

#include <QIcon>
#include <QDebug>
#include <QResource>
#include <QAbstractItemView>
#include <QTime>

#include <omp.h>
#include <pcl/common/pca.h>

#include "edit_flood_normals.h"
#include "utilities.h"
#include "layer.h"
#include "glarea.h"
#include "cloudmodel.h"


EditBrush::EditBrush()
{
    settings = new Settings();
    kNoise = settings->kNoise();
    editSample = new QAction(QIcon(":/images/brush.png"), "Floodfill (Normal stdev)", this);
    actionList << editSample;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

EditBrush::~EditBrush()
{
    delete settings;
}

void EditBrush::paintGL(CloudModel *, GLArea * glarea){

}

bool EditBrush::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    return true;
}


inline float clamp(float x, float a, float b)
{
    return x < a ? a : (x > b ? b : x);
}

void EditBrush::calcLocalNoise(CloudModel *cm){

    kNoise = settings->kNoise();

    QTime t;
    t.start();

    // For every value
    for(int i = 0; i < cm->cloud->size(); i++){

        boost::shared_ptr <std::vector<int> > kIdxs;
        kIdxs = boost::shared_ptr <std::vector<int> >(new std::vector<int>);
        vector<float> kDist;
        octree->nearestKSearch(i, kNoise , *kIdxs, kDist);

        std::vector<float> angles;
        angles.resize(kNoise);

        Eigen::Map<Eigen::Vector3f> current(cm->normals->at(i).data_n);

        float sumOfSquares = 0.0f;
        float sum = 0.0f;

        for(int idx : *kIdxs){

            Eigen::Map<Eigen::Vector3f> neighbour(cm->normals->at(idx).data_n);

            float cosine = neighbour.dot(current) /
                    neighbour.norm()*current.norm();

            cosine = clamp(cosine, 0.0f, 1.0f);

            // Normalised angle
            float angle = acos(cosine)/M_PI;
/*
            qDebug("Current: %f %f %f, Neighbour: %f %f %f",
                   cm->normals->at(i).data_n[0], cm->normals->at(i).data_n[1], cm->normals->at(i).data_n[2],
                   cm->normals->at(idx).data_n[0], cm->normals->at(idx).data_n[1], cm->normals->at(idx).data_n[2]);
            qDebug("Cosine: %f, Angle: %f", cosine, angle);
*/
            sum += angle;
            sumOfSquares += angle*angle;

        }

        float stdDev = sqrt( (sumOfSquares/kNoise) - pow(sum/kNoise, 2));

        noise_vals->at(i) = stdDev;
        /*
        qDebug("Sum of squares: %f, Sum: %f, N: %d", sumOfSquares, sum, kNoise);
        qDebug("Noise stdev: %f", stdDev);
        */
    }

    qDebug("Time to calc Noise metric: %d ms", t.elapsed());

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

        noise_vals = boost::shared_ptr<std::vector<float> >(new std::vector<float>());
        noise_vals->resize(cm->cloud->size());

        calcLocalNoise(cm);
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

inline float angularSimilarity(Eigen::Vector3f &a, Eigen::Vector3f &b){
    float cosineSimilarity = a.dot(b) / (a.norm()*b.norm());
    float angularSimlarity = acos(cosineSimilarity)/M_PI;
    return angularSimlarity;
}

void EditBrush::fill(int x, int y, float radius, int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea){

    Eigen::Vector3f p1, p2;

    QTime t;
    t.start();

    int min_index = pointPick(x, y, radius, source_idx, p1, p2, cm, glarea);

    qDebug("Time to pick point: %d ms", t.elapsed());

    if(min_index == -1)
        return;

    t.start();
    // Recalculate PCA if the K neighbourhood has changed
    if(kNoise != settings->kNoise())
        calcLocalNoise(cm);

    std::queue<int> myqueue;
    myqueue.push(min_index);
    int current;

    std::vector<int> & source = cm->layerList.layers[source_idx].index;
    std::vector<int> & dest = cm->layerList.layers[dest_idx].index;

    int K = settings->kConnectvity();
    float minNoise = settings->minNoise();
    float maxNoise = settings->maxNoise();

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

            //float diff = abs(noise_vals->at(current) - noise_vals->at(idx));

            float noise = noise_vals->at(current);

            qDebug("Noise %f. Min: %f, Max; %f", noise, minNoise, maxNoise);

            if(noise < minNoise || noise > maxNoise)
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


        if(dest_layer == -1 || dest_layer >= cm->layerList.layers.size()){
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

QWidget * EditBrush::getSettingsWidget(QWidget *){
    return settings;
}

Q_EXPORT_PLUGIN2(pnp_editbrush, EditBrush)
