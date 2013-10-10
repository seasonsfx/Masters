#include "plugins/markov/markov.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "plugins/markov/mincut.h"
#include <boost/serialization/shared_ptr.hpp>
#include <pcl/search/kdtree.h>

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

    myaction = new QAction(QIcon(":/markov.png"), "markov action", 0);

    connect(myaction,&QAction::triggered, [this] (bool on) {
        qDebug() << "Click!";
    });

    connect(myaction, SIGNAL(triggered()), this, SLOT(myFunc()));

    mw_->toolbar_->addAction(myaction);
}

void Markov::cleanup(){
    mw_->toolbar_->removeAction(myaction);
    delete myaction;
}

Markov::~Markov(){
    qDebug() << "Markov deleted";
}

void Markov::myFunc(){
    qDebug() << "Myfunc";

    // Need to subsample the cloud

    MinCut mc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr(cl_->active_.get(), boost::serialization::null_deleter());
    mc.setInputCloud(ptr);

    pcl::IndicesPtr source_indices(new std::vector<int>);

    // We need a layer for the region of interest

    // Need a layer for the pixels pinned to the fg

    // Need a layer for the pixels pinned to the bg

}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
