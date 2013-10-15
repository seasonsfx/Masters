#include "plugins/flood/flood.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QApplication>
#include <QEvent>
#include <QMouseEvent>
#include <QTime>

#include <set>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "plugins/normalestimation/normalestimation.h"
#include "utilities/pointpicker.h"
#include "utilities/utils.h"
#include "commands/select.h"

#include <pcl/kdtree/kdtree_flann.h>

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
}

void Flood::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->removeMenu(enable_, "Edit");
    delete enable_;
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

    glwidget_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));

    enable_->setChecked(true);

}

void Flood::disable() {
    is_enabled_ = false;
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
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

    int count = 0;
    int dist_count = 0;

    while (!flood_queue.empty()){
        current_idx = flood_queue.front(); flood_queue.pop();

        bool seen = !visited.insert(current_idx).second;

        if(seen)
            continue;

        count++;


        std::vector<int> idxs;
        std::vector<float> dists;
        search.radiusSearch(current_idx, radius, idxs, dists, max_nn);

        for (int idx : idxs) {
            pcl::Normal & n = (*normals)[idx];
            Eigen::Map<Eigen::Vector3f> normal(&n.normal_x);

            float dist = (normal-source_normal).norm();

            if(count < 10){
                qDebug() << "normal: " << normal.x() << normal.y() << normal.z();
                qDebug() << "Dist" << dist;
            }

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
    for(int big_idx = 0; big_idx < big_to_small.size(); big_idx++){
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

bool Flood::mouseReleaseEvent(QMouseEvent * event){
    qDebug() << event->x() << event->y();

    int idx = pick(event->x(), event->y(), glwidget_->width(),
                   glwidget_->height(), 1e-04,
                   glwidget_->camera_.projectionMatrix(),
                   glwidget_->camera_.modelviewMatrix(),
                   cl_->active_);

    if(idx == -1)
        return true;

    flood(idx);
    return true;
}

bool Flood::eventFilter(QObject *object, QEvent *event){

    // Bypass plugin via shift
    if(QApplication::keyboardModifiers() == Qt::SHIFT)
        return false;

    switch(event->type()){
    case QEvent::MouseButtonRelease:
        return mouseReleaseEvent(static_cast<QMouseEvent*>(event));
    default:
        return false;
    }
}


Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
