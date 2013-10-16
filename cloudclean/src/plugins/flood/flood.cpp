#include "plugins/flood/flood.h"

#include <functional>
#include <algorithm>
#include <set>

#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QApplication>
#include <QEvent>
#include <QMouseEvent>
#include <QTime>

#include <boost/serialization/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/principal_curvatures.h>

#define PCL_NO_PRECOMPILE
#include <pcl/segmentation/region_growing.h>
#undef PCL_NO_PRECOMPILE

#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "commands/newlayer.h"
#include "pluginsystem/core.h"
#include "plugins/normalestimation/normalestimation.h"
#include "utilities/picker.h"
#include "utilities/utils.h"
#include "utilities/cv.h"
#include "commands/select.h"



QString Flood::getName(){
    return "flood";
}

void Flood::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    std::function<void(int)> func = std::bind(&Flood::flood, this, std::placeholders::_1);
    picker_ = new Picker(glwidget_, cl_, func);

}

void Flood::initialize2(PluginManager * pm) {
    ne_ = pm->findPlugin<NormalEstimator>();
    if (ne_ == nullptr) {
        qDebug() << "Normal estimator plugin needed for normal viz";
        return;
    }

    enable_ = new QAction(QIcon(":/images/flood.jpg"), "Floodfill", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);
    is_enabled_ = false;

    mw_->addMenu(enable_, "Edit");
    mw_->toolbar_->addAction(enable_);

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));

    mw_->toolbar_->addAction(enable_);

    global_flood_ = new QAction(QIcon(":/images/flood2.jpg"), "Global floodfill", 0);
    connect(global_flood_, &QAction::triggered, this, &Flood::global_flood2);
    mw_->toolbar_->addAction(global_flood_);
}

void Flood::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->removeMenu(enable_, "Edit");
    delete enable_;
    delete global_flood_;
}

Flood::~Flood(){
}

void Flood::enable() {
    qDebug() << "enablebling";

    if(is_enabled_){
        disable();
        return;
    }

    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
    tabs->setCurrentWidget(glwidget_);

    is_enabled_ = true;
    enable_->setChecked(true);

    emit enabling();

    //glwidget_->installEventFilter(this);
    glwidget_->installEventFilter(picker_);

    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));

    enable_->setChecked(true);

}

void Flood::disable() {
    is_enabled_ = false;
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(picker_);
}

boost::shared_ptr<std::vector<int> > Flood::getLayerIndices() {
    boost::shared_ptr<std::vector<int>> indices = boost::make_shared<std::vector<int>>();

    boost::shared_ptr<PointCloud> active = cl_->active_;

    if(ll_->selection_.size() == 0){
        for(uint i = 0; i < active->size(); i++) {
            indices->push_back(i);
        }
        return indices;
    }

    // get labels in selected layers
    std::set<uint16_t> selected_labels;

    for(boost::weak_ptr<Layer> wl: ll_->selection_) {
        boost::shared_ptr<Layer> l = wl.lock();
        if(l == nullptr)
            continue;

        for(uint16_t label: l->getLabelSet()) {
            selected_labels.insert(label);
        }
    }

    auto in = [] (uint16_t label, std::set<uint16_t> labels) {
        for(uint16_t l : labels){
            if(l == label)
                return true;
        }
        return false;
    };

    for(uint i = 0; i < active->size(); i++) {
        if(in(active->labels_[i], selected_labels)){
            indices->push_back(i);
        }
    }

    return indices;
}

void Flood::flood(int source_idx){

    float max_dist = 1.0f;
    int max_nn = 8;
    float radius = 0.10f;

    QTime t;
    t.start();

    // get normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cl_->active_);

    // zip and downsample
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr smallcloud = zipNormals(cl_->active_, normals);
    std::vector<int> big_to_small;
    smallcloud = octreeDownsample(smallcloud.get(), 0.05, big_to_small);

/*
    // write normals back so we can see
    for(int big_idx = 0; big_idx < normals->size(); ++big_idx){
        int small_idx = big_to_small[big_idx];
        pcl::Normal & n = normals->points[big_idx];
        pcl::PointXYZINormal & pn = smallcloud->points[small_idx];
        n.getNormalVector4fMap() = pn.getNormalVector4fMap();
    }
*/

    pcl::PointXYZINormal & n = (*smallcloud)[big_to_small[source_idx]];
    Eigen::Map<Eigen::Vector3f> source_normal(&n.normal_x);

    qDebug() << "Source normal: " << source_normal.x() << source_normal.y() << source_normal.z();

    std::queue<int> flood_queue;
    flood_queue.push(big_to_small[source_idx]);
    int current_idx;

    //boost::shared_ptr<std::vector<int> > indices = getLayerIndices();

    //Octree search = *(cl_->active_->octree());
    pcl::KdTreeFLANN<pcl::PointXYZINormal> search;
    search.setInputCloud(smallcloud);

    std::set<int> visited;

    int dist_count = 0;

    while (!flood_queue.empty()){
        current_idx = flood_queue.front(); flood_queue.pop();

        bool seen = !visited.insert(current_idx).second;

        if(seen)
            continue;

        std::vector<int> idxs;
        std::vector<float> dists;
        search.radiusSearch(current_idx, radius, idxs, dists, max_nn);

        for (int idx : idxs) {
            pcl::PointXYZINormal & n = (*smallcloud)[idx];
            Eigen::Map<Eigen::Vector3f> normal(&n.normal_x);

            float dist = (normal-source_normal).norm();

            if(dist > max_dist) {
                if(dist_count++ < 10){
                    qDebug() << "too big:" << dist;
                }
                continue;
            }

            flood_queue.push(idx);
        }
    }

    // create selection
    boost::shared_ptr<std::vector<int> > selected = boost::make_shared<std::vector<int> >();

    // map back to original cloud
    for(size_t big_idx = 0; big_idx < big_to_small.size(); big_idx++){
        int small_idx = big_to_small[big_idx];
        bool seen = visited.find(small_idx) != visited.end();
        if(seen){
            selected->push_back(big_idx);
        }
    }

    core_->us_->beginMacro("Normal fill");
    core_->us_->push(new Select(cl_->active_, selected));
    core_->us_->endMacro();

    qDebug("Time to fill : %d ms", t.elapsed());
}

void Flood::global_flood(){
    qDebug() << "Booya!";

    // get normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cl_->active_);

    // zip and downsample
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr smallcloud = zipNormals(cl_->active_, normals);
    std::vector<int> big_to_small;
    smallcloud = octreeDownsample(smallcloud.get(), 0.05, big_to_small);

    pcl::search::Search<pcl::PointXYZINormal>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZINormal> > (new pcl::search::KdTree<pcl::PointXYZINormal>);

    pcl::RegionGrowing<pcl::PointXYZINormal, pcl::PointXYZINormal> reg;
    reg.setMinClusterSize (1000);
    reg.setMaxClusterSize (10000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (10);
    reg.setInputCloud (smallcloud);
    //reg.setIndices (indices);
    reg.setInputNormals (smallcloud);

    reg.setCurvatureTestFlag(false);
    reg.setSmoothnessThreshold (DEG2RAD(30.0));
    //reg.setCurvatureThreshold (0.5);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);


    // create small to big map
    std::vector<std::vector<int>> small_to_big(smallcloud->size());

    for(size_t big_idx = 0; big_idx++ < big_to_small.size(); big_idx++) {
        int small_idx = big_to_small[big_idx];
        small_to_big[small_idx].push_back(big_idx);
    }

    // create new layers from clusters
    core_->us_->beginMacro("Global flood fill");
    for(pcl::PointIndices & idxs : clusters){
        boost::shared_ptr<std::vector<int>> big_idxs = boost::make_shared<std::vector<int>>();
        for(int small_idx : idxs.indices){
            for(int big_idx : small_to_big[small_idx]) {
                big_idxs->push_back(big_idx);
            }
        }
        NewLayer * nl = new NewLayer(cl_->active_, big_idxs, ll_);
        core_->us_->push(nl);
    }
    core_->us_->endMacro();
}

void Flood::global_flood2(){
    float max_dist = 1.0f;
    int max_nn = 8;
    float radius = 0.10f;
    int min_region = 1000;

    float subsample_density = 0.05;

    //// downsample
    // get normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cl_->active_);

    // zip and downsample
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr smallcloud = zipNormals(cl_->active_, normals);
    std::vector<int> big_to_small;
    smallcloud = octreeDownsample(smallcloud.get(), subsample_density, big_to_small);

    // create small to big map
    std::vector<std::vector<int>> small_to_big(smallcloud->size());

    for(size_t big_idx = 0; big_idx++ < big_to_small.size(); big_idx++) {
        int small_idx = big_to_small[big_idx];
        small_to_big[small_idx].push_back(big_idx);
    }

    //// compute curvature

    // Setup the principal curvatures computation


    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

    principal_curvatures_estimation.setInputCloud (smallcloud);
    principal_curvatures_estimation.setInputNormals (smallcloud);

    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal>);
    principal_curvatures_estimation.setSearchMethod (tree);
    principal_curvatures_estimation.setRadiusSearch (0.5);

    // Actually compute the principal curvatures
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principal_curvatures_estimation.compute (*principal_curvatures);

    // Filter out NaNs
    int nans = 0;
    for(pcl::PrincipalCurvatures & pc : principal_curvatures->points) {
        if(pc.principal_curvature_x != pc.principal_curvature_x){
            pc.principal_curvature_x = 0;
            pc.principal_curvature_y = 0;
            pc.principal_curvature_z = 0;
            nans++;
        }
    }


    // PCA
    //boost::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(smallcloud.get(), 0.5f, 0);


    // Set largest curvature on normal
    for(size_t i = 0; i < smallcloud->size(); i++){
        // assume curvature x is the biggests
        float curv = principal_curvatures->points[i].principal_curvature_x;
        if(!pcl_isnan(curv))
            smallcloud->points[i].curvature = curv;
        else
            smallcloud->points[i].curvature = INFINITY;
    }

    //// set seeds

    float curvature_max = 0.5f;
    std::vector<int> seeds;

    int run = 10;

    for(size_t i = 0; i < smallcloud->size(); i++){
        if(smallcloud->points[i].curvature < curvature_max)
            seeds.push_back(i);
        else {
            if(--run > 0){
                qDebug() << smallcloud->points[i].curvature;
            }
        }
    }

    // sort seeds

    std::sort(seeds.begin(), seeds.end(), [&smallcloud] (const int & a, const int & b) -> bool {
        return smallcloud->points[a].curvature < smallcloud->points[b].curvature;
    });

    //// Run the fill

    // keep track of points that are in regions already
    std::set<int> seen;

    auto fill = [&] (int source_idx) {

        std::vector<int> region;

        Eigen::Map<Eigen::Vector3f> source_normal = (*smallcloud)[source_idx].getNormalVector3fMap();

        std::queue<int> flood_queue;
        flood_queue.push(big_to_small[source_idx]);
        int current_idx;

        pcl::KdTreeFLANN<pcl::PointXYZINormal> search;
        search.setInputCloud(smallcloud);

        // debugging variable
        int dist_count = 0;

        while (!flood_queue.empty()){
            current_idx = flood_queue.front(); flood_queue.pop();

            bool visited = !seen.insert(current_idx).second;

            if(visited)
                continue;

            region.push_back(current_idx);

            // add neighbours:

            std::vector<int> idxs;
            std::vector<float> dists;
            //search.nearestKSearch(current_idx, radius, k, dists, max_nn);
            search.radiusSearch(current_idx, radius, idxs, dists, max_nn);

            for (int idx : idxs) {
                Eigen::Map<Eigen::Vector3f> normal = (*smallcloud)[idx].getNormalVector3fMap();

                float dist = (normal-source_normal).norm();

                // skip points out of range
                if(dist > max_dist) {
                    if(dist_count++ < 10){
                        qDebug() << "too big:" << dist;
                    }
                    continue;
                }

                flood_queue.push(idx);
            }
        }

        return region;

    };

    core_->us_->beginMacro("Global flood fill 2");

    for(size_t idx = 0; idx < seeds.size(); idx++) {
        int seed_idx = seeds[idx];

        // Skip seeds already visited
        if(seen.insert(seed_idx).second == true)
            continue;

        std::vector<int> region = fill(seed_idx);

        // Remove the region from the seen points if the region is too small
        if(region.size() < min_region) {
            for(int re_idx : region) {
                seen.erase(seen.find(re_idx));
            }
            continue;
        }

        // Create a big layer
        boost::shared_ptr<std::vector<int>> big_idxs = boost::make_shared<std::vector<int>>();
        for(int small_idx : region){
            for(int big_idx : small_to_big[small_idx]) {
                big_idxs->push_back(big_idx);
            }
        }
        NewLayer * nl = new NewLayer(cl_->active_, big_idxs, ll_);
        core_->us_->push(nl);

    }

    core_->us_->endMacro();

    // for each point in sorted curvatures:
    // flood lowest curvature
    // remove flooded points form sorted list


}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
