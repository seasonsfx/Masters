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

//#define PCL_NO_PRECOMPILE
//#include <pcl/segmentation/region_growing.h>
//#undef PCL_NO_PRECOMPILE

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

    //global_flood_ = new QAction(QIcon(":/images/flood2.jpg"), "Global floodfill", 0);
    //connect(global_flood_, &QAction::triggered, this, &Flood::global_flood);
    //mw_->toolbar_->addAction(global_flood_);

    global_flood2_ = new QAction(QIcon(":/images/flood2.jpg"), "Global floodfill 2", 0);
    connect(global_flood2_, &QAction::triggered, this, &Flood::global_flood2);
    mw_->toolbar_->addAction(global_flood2_);

}

void Flood::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->removeMenu(enable_, "Edit");
    delete enable_;
    //mw_->toolbar_->removeAction(global_flood_);
    //delete global_flood_;
    mw_->toolbar_->removeAction(global_flood2_);
    delete global_flood2_;
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

    if(ll_->getSelection().size() == 0){
        for(uint i = 0; i < active->size(); i++) {
            indices->push_back(i);
        }
        return indices;
    }

    // get labels in selected layers
    std::set<uint16_t> selected_labels;

    for(boost::weak_ptr<Layer> wl: ll_->getSelection()) {
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

/*
void Flood::global_flood(){
    qDebug() << "Booya!";

    // get normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cl_->active_);

    // zip and downsample
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptr(cl_->active_.get(), boost::serialization::null_deleter());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr smallcloud = zipNormals(ptr, normals);
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

    for(size_t big_idx = 0; big_idx < big_to_small.size(); big_idx++) {
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
*/

void Flood::global_flood2(){
    float max_dist = 1.0f;
    int max_nn = 8;
    float radius = 0.10f;
    size_t min_region = 100;

    float subsample_density = 0.05;
    float seed_curvature_max = 0.02f;


    //// downsample
    // get normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cl_->active_);

    // zip and downsample
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptr(cl_->active_.get(), boost::serialization::null_deleter());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr smallcloud = zipNormals(ptr, normals);
    std::vector<int> big_to_small;
    smallcloud = octreeDownsample(smallcloud.get(), subsample_density, big_to_small);

    // create small to big map
    std::vector<std::vector<int>> small_to_big(smallcloud->size());

    for(size_t big_idx = 0; big_idx < big_to_small.size(); big_idx++) {
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


    // Set largest curvature on normal
    for(size_t i = 0; i < smallcloud->size(); i++){
        // assume curvature x is the biggests
        pcl::PrincipalCurvatures & pc = (*principal_curvatures)[i];

        float mean_curv = (pc.pc1 + pc.pc2)/2;
        smallcloud->points[i].curvature = mean_curv;
    }

    //// set seeds
    ///
    std::vector<int> seeds;

    for(size_t i = 0; i < smallcloud->size(); i++){
        if(smallcloud->points[i].curvature < seed_curvature_max /*&& smallcloud->points[i].curvature > 0*/)
            seeds.push_back(i);
    }

    // sort seeds

    std::sort(seeds.begin(), seeds.end(), [&smallcloud] (const int & a, const int & b) -> bool {
        return smallcloud->points[a].curvature < smallcloud->points[b].curvature;
    });

    qDebug() << "Number of seeds: " << seeds.size();

    int debugs = 10;
    for(int seed : seeds){
        if(--debugs > 0){
            qDebug() << smallcloud->points[seed].curvature;
        }
        else {
            break;
        }
    }

    //// Run the fill

    // keep track of points that are in regions already
    std::set<int> seen;

    pcl::KdTreeFLANN<pcl::PointXYZINormal> search;
    search.setInputCloud(smallcloud);

    // debugging variable
    int debug_dist_count = 0;

    auto fill = [&] (int source_idx) {

        std::vector<int> region;

        std::vector<int> idxs;
        std::vector<float> dists;
        search.radiusSearch(source_idx, radius, idxs, dists, max_nn);

        Eigen::Vector3f source_normal(0, 0, 0);

        for(int idx : idxs) {
            source_normal += (*smallcloud)[idx].getNormalVector3fMap();
        }

        source_normal /= idxs.size();

        //Eigen::Map<Eigen::Vector3f> source_normal = (*smallcloud)[source_idx].getNormalVector3fMap();

        std::queue<int> flood_queue;
        flood_queue.push(source_idx);
        int current_idx = -1;

        while (!flood_queue.empty()){
            current_idx = flood_queue.front(); flood_queue.pop();

            bool not_seen = seen.insert(current_idx).second;

            if(!not_seen)
                continue;

            region.push_back(current_idx);

            // add neighbours:

            std::vector<int> idxs;
            std::vector<float> dists;
            search.radiusSearch(current_idx, radius, idxs, dists, max_nn);

            for (int idx : idxs) {
                Eigen::Map<Eigen::Vector3f> normal = (*smallcloud)[idx].getNormalVector3fMap();

                float dist = (normal-source_normal).norm();

                if(debug_dist_count++ < 10){
                    qDebug() << "dist:" << dist;
                }

                if(dist != dist) {
                    qDebug() << "source normal map" << source_normal[0] << source_normal[1] << source_normal[2];
                    qDebug() << "normal map" << normal[0] << normal[1] << normal[2];
                    qDebug() << "normal.." << (*smallcloud)[idx].normal_x << (*smallcloud)[idx].normal_y << (*smallcloud)[idx].normal_z;
                }

                // skip points out of range
                if(dist > max_dist || dist != dist) {
                    continue;
                }

                flood_queue.push(idx);
            }
        }

        return region;
    };

    core_->us_->beginMacro("Global flood fill 2");

    for(uint idx = 0; idx < seeds.size(); idx++) {
        int seed_idx = seeds[idx];

        std::vector<int> region = fill(seed_idx);




        // Remove the region from the seen points if the region is too small
        if(region.size() < min_region) {
            for(int re_idx : region) {
                seen.erase(seen.find(re_idx));
            }
            qDebug() << "Deleted region of size" << region.size();

            continue;
        }

        qDebug() << "Curvature" << smallcloud->points[seed_idx].curvature;
        qDebug() << "Region: " << region.size();

        qDebug() << "Big enough";

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
