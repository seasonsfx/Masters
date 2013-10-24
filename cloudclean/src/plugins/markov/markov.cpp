#include "plugins/markov/markov.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QMessageBox>
#include <functional>
#include <boost/serialization/shared_ptr.hpp>
#include <pcl/search/kdtree.h>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "plugins/markov/mincut.h"
#include "utilities/utils.h"
#include "utilities/picker.h"
#include "utilities/cv.h"

#include "data.h"
#include "onlinetree.h"
#include "onlinerf.h"

QString Markov::getName(){
    return "markov";
}

void Markov::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    enable_ = new QAction(QIcon(":/markov.png"), "markov action", 0);
    enable_->setCheckable(true);

    connect(enable_,&QAction::triggered, [this] (bool on) {
        graphcut();
    });

    mw_->toolbar_->addAction(enable_);
    std::function<void(int)> func = std::bind(&Markov::graphcut, this, std::placeholders::_1);


    forrest_action_ = new QAction(QIcon(":/randomforest.png"), "forrest action", 0);
    connect(forrest_action_, &QAction::triggered, [this] (bool on) {
        randomforest();
    });

    picker_ = new Picker(glwidget_, cl_, func);
    enabled_ = false;
    fg_idx_ = -1;
}

void Markov::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    delete enable_;
    delete picker_;
    delete forrest_action_;
}

Markov::~Markov(){
    qDebug() << "Markov deleted";
}

void Markov::enable() {
    if(enabled_){
        disable();
        return;
    }

    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
    tabs->setCurrentWidget(glwidget_);
    enable_->setChecked(true);

    emit enabling();

    glwidget_->installEventFilter(picker_);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));

    enabled_ = true;
    // Let the user know what to do
    QMessageBox::information(nullptr, tr("Select foreground"),
                    tr("Select the center of an object..."),
                    QMessageBox::Ok, QMessageBox::Ok);


}

void Markov::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(picker_);
    enabled_ = false;
}

void Markov::graphcut(int idx){
    qDebug() << "Myfunc";
/*
    if(fg_idx_ == -1) {
        fg_idx_ = idx;
        QMessageBox::information(nullptr, tr("Select background"),
                        tr("Select background"),
                        QMessageBox::Ok, QMessageBox::Ok);
        return;
    }
*/
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;


    // Downsample
    std::vector<int> pca_idxs;

    pcl::PointCloud<pcl::PointXYZI>::Ptr smaller_cloud = octreeDownsample(cloud.get(), 0.02, pca_idxs);

    // PCA
    boost::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(smaller_cloud.get(), 0.2f, 0);

    // Determine veg
    std::vector<bool> likely_veg(smaller_cloud->size());

    for(uint i = 0; i < pca->size(); i++) {
        Eigen::Vector3f eig = (*pca)[i];

        // If not enough neighbours
        if(eig[1] < eig[2]) {
            likely_veg[i] = false;
            continue;
        }

        float eig_sum = eig.sum();

        eig /= eig_sum;

        float fudge_factor = 5.0f;
        if(eig[1] < 0.05 * fudge_factor || eig[2] < 0.01 * fudge_factor) {
            likely_veg[i] = false;
        } else {
            likely_veg[i] = true;
        }

    }



    // Downsample for graph cut
    std::vector<int> big_to_small_map;
    smaller_cloud = octreeDownsample(cloud.get(), 0.1, big_to_small_map);

    std::set<int> foreground_points;

    // Map veg to foreground points in 2nd downsampled cloud
    for(uint i = 0; i < cloud->size(); i++) {
        uint pca_idx = pca_idxs[i];
        if(likely_veg[pca_idx] == true){
            uint small_idx = big_to_small_map[i];
            foreground_points.insert(small_idx);
        }
    }


    // Set up graph cut

    MinCut mc;
    mc.setInputCloud(smaller_cloud);


    //foreground_points.push_back(big_to_small_map[fg_idx_]);

    //std::vector<int> background_points;
    //background_points.push_back(big_to_small_map[idx]);

    double radius = 3.0;
    double sigma = 0.25;
    int neigbours = 10;
    double source_weight = 0.8;

    mc.setForegroundPoints (foreground_points);
    //mc.setBackgroundPoints(background_points);
    mc.setRadius (radius);
    mc.setSigma (sigma);
    mc.setNumberOfNeighbours (neigbours);
    mc.setSourceWeight (source_weight);

    std::vector<pcl::PointIndices> clusters;
    mc.extract (clusters);

    auto select = boost::make_shared<std::vector<int>>();

    // GAH!!!! inefficient

    std::vector<bool> small_idxs_selected(smaller_cloud->size(), false);
    for(int idx : clusters[1].indices){
        small_idxs_selected[idx] = true;
    }

    for(size_t i = 0; i < big_to_small_map.size(); i++) {
        int idx = big_to_small_map[i];
        if(small_idxs_selected[idx]) {
            select->push_back(i);
        }
    }

    //std::copy(clusters[1].indices.begin(), clusters[1].indices.end(), select->begin());

    Select * selectCmd = new Select(cl_->active_, select);



    core_->us_->beginMacro("Markov min cut");
    core_->us_->push(selectCmd);
    core_->us_->endMacro();
    core_->cl_->updated();
    core_->mw_->stopBgAction("Cut completed.");


    // We need a layer for the region of interest

    // Need a layer for the pixels pinned to the fg

    // Need a layer for the pixels pinned to the bg
    fg_idx_ = -1;
    disable();
}

void Markov::randomforest(){
    qDebug() << "Random forest";

    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;


    // Downsample
    std::vector<int> pca_idxs;

    pcl::PointCloud<pcl::PointXYZI>::Ptr smaller_cloud = octreeDownsample(cloud.get(), 0.02, pca_idxs);

    // PCA
    boost::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(smaller_cloud.get(), 0.2f, 0);

    // Determine veg
    std::vector<bool> likely_veg(smaller_cloud->size());

    for(uint i = 0; i < pca->size(); i++) {
        Eigen::Vector3f eig = (*pca)[i];

        // If not enough neighbours
        if(eig[1] < eig[2]) {
            likely_veg[i] = false;
            continue;
        }

        float eig_sum = eig.sum();

        eig /= eig_sum;

        float fudge_factor = 5.0f;
        if(eig[1] < 0.05 * fudge_factor || eig[2] < 0.01 * fudge_factor) {
            likely_veg[i] = false;
        } else {
            likely_veg[i] = true;
        }

    }

    // Downsample for graph cut
    std::vector<int> big_to_small_map;
    smaller_cloud = octreeDownsample(cloud.get(), 0.1, big_to_small_map);

    std::set<int> foreground_points;

    // Map veg to foreground points in 2nd downsampled cloud
    for(uint i = 0; i < cloud->size(); i++) {
        uint pca_idx = pca_idxs[i];
        if(likely_veg[pca_idx] == true){
            uint small_idx = big_to_small_map[i];
            foreground_points.insert(small_idx);
        }
    }


    // Set up graph cut

    MinCut mc;
    mc.setInputCloud(smaller_cloud);


    //foreground_points.push_back(big_to_small_map[fg_idx_]);

    //std::vector<int> background_points;
    //background_points.push_back(big_to_small_map[idx]);

    double radius = 3.0;
    double sigma = 0.25;
    int neigbours = 10;
    double source_weight = 0.8;

    mc.setForegroundPoints (foreground_points);
    //mc.setBackgroundPoints(background_points);
    mc.setRadius (radius);
    mc.setSigma (sigma);
    mc.setNumberOfNeighbours (neigbours);
    mc.setSourceWeight (source_weight);

    std::vector<pcl::PointIndices> clusters;
    mc.extract (clusters);

    auto select = boost::make_shared<std::vector<int>>();

    // GAH!!!! inefficient

    std::vector<bool> small_idxs_selected(smaller_cloud->size(), false);
    for(int idx : clusters[1].indices){
        small_idxs_selected[idx] = true;
    }

    for(size_t i = 0; i < big_to_small_map.size(); i++) {
        int idx = big_to_small_map[i];
        if(small_idxs_selected[idx]) {
            select->push_back(i);
        }
    }

    //std::copy(clusters[1].indices.begin(), clusters[1].indices.end(), select->begin());

    Select * selectCmd = new Select(cl_->active_, select);



    core_->us_->beginMacro("Markov min cut");
    core_->us_->push(selectCmd);
    core_->us_->endMacro();
    core_->cl_->updated();
    core_->mw_->stopBgAction("Cut completed.");


    // We need a layer for the region of interest

    // Need a layer for the pixels pinned to the fg

    // Need a layer for the pixels pinned to the bg
    fg_idx_ = -1;
    disable();
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
