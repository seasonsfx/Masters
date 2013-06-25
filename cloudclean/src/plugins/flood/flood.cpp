#include "plugins/flood/flood.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QApplication>
#include <QEvent>
#include <QMouseEvent>
#include <QTime>

#include <boost/serialization/shared_ptr.hpp>

#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "plugins/normalestimation/normalestimation.h"
#include "utilities/pointpicker.h"
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

std::shared_ptr<std::vector<int> > Flood::getLayerIndices() {
    std::shared_ptr<std::vector<int>> indices = std::make_shared<std::vector<int>>();

    std::shared_ptr<PointCloud> active = cl_->active_;

    if(ll_->selection_.size() == 0){
        for(int i = 0; i < active->size(); i++) {
            indices->push_back(i);
        }
        return indices;
    }

    // get labels in selected layers
    std::set<uint16_t> selected_labels;

    for(std::weak_ptr<Layer> wl: ll_->selection_) {
        std::shared_ptr<Layer> l = wl.lock();
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

    for(int i = 0; i < active->size(); i++) {
        if(in(active->labels_[i], selected_labels)){
            indices->push_back(i);
        }
    }

    return indices;
}

void Flood::flood(int source_idx){
    QTime t;
    t.start();

    // get normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cl_->active_);

    Eigen::Map<Eigen::Vector3f> source_normal(&((*normals)[source_idx].data_c[0]));

    std::queue<int> index_queue;
    index_queue.push(source_idx);
    int current_idx;

    std::shared_ptr<std::vector<int> > indices = getLayerIndices();

    //Octree::Ptr octree = cl_->active_->octree();
    pcl::KdTreeFLANN<pcl::PointXYZI> search;
    search.setInputCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr(cl_->active_.get(), boost::serialization::null_deleter()));

    double radius = 0.05;
    int max_nn = 4;

    std::shared_ptr<std::vector<int> > selected = std::make_shared<std::vector<int> >();

    while (!index_queue.empty()){
        current_idx = index_queue.front(); index_queue.pop();

        selected->push_back(current_idx);

        std::vector<int> idxs;
        std::vector<float> dists;
        search.radiusSearch(source_idx, radius, idxs, dists, max_nn);

        for (int idx : idxs) {
            Eigen::Map<Eigen::Vector3f> normal(&((*normals)[idx].data_c[0]));

            float dist = (normal-source_normal).norm();

            //qDebug("Noise %f. Min: %f, Max; %f", noise, minNoise, maxNoise);

            if(dist > 0.5)
                continue;

            index_queue.push(idx);
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
