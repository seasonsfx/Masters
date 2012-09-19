#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include <GL/glu.h>

#include <QIcon>
#include <QDebug>
#include <QResource>
#include <QAbstractItemView>
#include <QTime>

#include "edit_mincut.h"
//#include "utilities.h"
#include "layer.h"
#include "glarea.h"
#include "cloudmodel.h"
#include "mincut.h"


EditPlugin::EditPlugin()
{
    lasso_active = false;
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
    lasso_active = !lasso_active;
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
    lasso.clear();
    return true;
}


bool EditPlugin::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){

    return true;
}

bool EditPlugin::mouseMoveEvent (QMouseEvent *event, CloudModel *, GLArea * glarea){

    /// Save the last known normalised mouse positionb
    if(glarea->moved){
        normalised_mouse_loc = glarea->normalized_mouse(
                    event->x(), event->y());
        if(lasso_active)
            glarea->updateGL();
    }

    return true;
}

bool EditPlugin::mouseReleaseEvent(QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    if (!glarea->moved && event->button() == Qt::LeftButton){
        // Single selection switched on, on edit start

        if(!lasso_active && lasso.getPolygon().size() > 3){

            int source_layer = -1;

            // Sets source layer as the one selected
            for(int i = 0; i < cm->layerList.layers.size(); i++){
                Layer & l = cm->layerList.layers[i];
                if(l.active && l.visible){
                    source_layer = i;
                    l.sync();
                    break;
                }
            }

            // If the destination layer does not exist or has been deleted, fix it
            if(dest_layer == -1 || dest_layer >= cm->layerList.layers.size()){
                cm->layerList.newLayer();
                dest_layer = cm->layerList.layers.size()-1;
            }


            qDebug("segmenting????????????????????");
            segment(source_layer, dest_layer, cm, glarea);
            lasso.clear();
        }
        else if(lasso_active){
            lasso.addPoint(this->normalised_mouse_loc);
        }

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

Eigen::Vector3f unProjectNDC(const Eigen::Matrix4f & proj,
                             const Eigen::Matrix4f & mv,
                             const float z,
                             const Eigen::Vector2f p){

    Eigen::Vector4f screen_p(p.x(), p.y(), z, 1.0f);
    Eigen::Vector4f unprojected_p = (proj * mv).inverse() * screen_p;
    unprojected_p /= unprojected_p[3];
    assert(unprojected_p.w() != 0.0f);
    return unprojected_p.head<3>();
}

// Unproject from 2D NDC space
std::vector<Eigen::Vector3f> unProjectPolygonNDC(
                                const Eigen::Matrix4f & proj,
                                const Eigen::Matrix4f & mv,
                                const float z,
                                const std::vector<Eigen::Vector2f> inpoly){

    std::vector<Eigen::Vector3f> unprojected_polygon;

    for(Eigen::Vector2f p: inpoly){
        Eigen::Vector4f screen_p(p.x(), p.y(), z, 1.0f);
        // Unproject
        Eigen::Vector4f unprojected_p = (proj * mv).inverse() * screen_p;
        unprojected_p /= unprojected_p[3];
        assert(unprojected_p.w() != 0.0f);
        unprojected_polygon.push_back(unprojected_p.head<3>());
    }

    return unprojected_polygon;
}

Eigen::Vector2f centoid(const std::vector<Eigen::Vector2f> polygon){
    float x = 0, y = 0;

    float signedArea = 0;

    for(int i = 0; i < polygon.size(); i++){
        Eigen::Vector2f p1 = polygon[i];
        Eigen::Vector2f p2 = polygon[(i+1)%polygon.size()];

        float a = p1.x()*p2.y() - p2.x()-p1.y();
        signedArea += a;
        x += (p1.x() + p2.x())*a;
        y += (p1.y() + p2.y())*a;
    }

    x/=(6*signedArea);
    y/=(6*signedArea);
    return Eigen::Vector2f(x,y);
}

void EditPlugin::segment(int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea){


    // Fetch point indices inside the lasso
    Eigen::Matrix4f ndc_trans = glarea->camera.projectionMatrix().matrix() * glarea->camera.modelviewMatrix().matrix();
    std::vector<int> & source_layer = cm->layerList.layers[source_idx].index;
    std::vector<int> & dest_layer = cm->layerList.layers[dest_idx].index;
    pcl::IndicesPtr inside_lasso_indices(new std::vector<int>);
    lasso.getIndices(ndc_trans, &*cm->cloud, source_layer, *inside_lasso_indices);

    MinCut seg;
    seg.setInputCloud(cm->cloud);
    seg.setIndices(inside_lasso_indices);
    auto polygon = lasso.getPolygon();

    float z = 1.0f;

    std::vector<Eigen::Vector3f> poly3d = unProjectPolygonNDC(
                                    glarea->camera.projectionMatrix().matrix(),
                                    glarea->camera.modelviewMatrix().matrix(),
                                    z,
                                    polygon);


    Eigen::Vector2f centoid2d = centoid(polygon);
    Eigen::Vector3f centoid3d = unProjectNDC(
                glarea->camera.projectionMatrix().matrix(),
                glarea->camera.modelviewMatrix().matrix(),
                z,
                centoid2d);


    seg.setBoundingPolygon(poly3d, centoid3d);

    seg.setSigma (settings->sigma()); // Density me thinks
                                      // try set this dynamically
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
        cm->layerList.layers[source_idx].index[idx] = idx;
    }

    for(int idx : clusters[1].indices){
        cm->layerList.layers[dest_idx].index[idx] = idx;
    }

    cm->layerList.layers[source_idx].copyToGPU();
    cm->layerList.layers[dest_idx].copyToGPU();

}

Q_EXPORT_PLUGIN2(pnp_editbrush, EditPlugin)
