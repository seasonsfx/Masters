#include "plugins/stub/stub.h"
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

#include <pcl/search/kdtree.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/boost.h>
#include <pcl/search/search.h>
#include <cmath>

typedef boost::adjacency_list_traits< boost::vecS, boost::vecS, boost::directedS > Traits;

typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::directedS,
                             boost::property< boost::vertex_name_t, std::string,
                               boost::property< boost::vertex_index_t, long,
                                 boost::property< boost::vertex_color_t, boost::default_color_type,
                                   boost::property< boost::vertex_distance_t, long,
                                     boost::property< boost::vertex_predecessor_t, Traits::edge_descriptor > > > > >,
                             boost::property< boost::edge_capacity_t, double,
                               boost::property< boost::edge_residual_capacity_t, double,
                                 boost::property< boost::edge_reverse_t, Traits::edge_descriptor > > > > mGraph;

typedef boost::property_map< mGraph, boost::edge_capacity_t >::type CapacityMap;

typedef boost::property_map< mGraph, boost::edge_reverse_t>::type ReverseEdgeMap;

typedef Traits::vertex_descriptor VertexDescriptor;

typedef boost::graph_traits< mGraph >::edge_descriptor EdgeDescriptor;

typedef boost::graph_traits< mGraph >::out_edge_iterator OutEdgeIterator;

typedef boost::graph_traits< mGraph >::vertex_iterator VertexIterator;

typedef boost::property_map< mGraph, boost::edge_residual_capacity_t >::type ResidualCapacityMap;

typedef boost::property_map< mGraph, boost::vertex_index_t >::type IndexMap;

typedef boost::graph_traits< mGraph >::in_edge_iterator InEdgeIterator;


QString Stub::getName(){
    return "stub";
}

void Stub::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    myaction = new QAction(QIcon(":/images/stub.png"), "Stub", 0);

    connect(myaction,&QAction::triggered, [this] (bool on) {
        qDebug() << "Click!";
    });

    connect(myaction, SIGNAL(triggered()), this, SLOT(myFunc()));
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));

    mw_->toolbar_->addAction(myaction);

    //
    pcl::search::KdTree<pcl::PointXYZI> wee;
    pcl::search::Search <pcl::PointXYZI> * KdTree;
    boost::shared_ptr<mGraph> graph_;
    graph_ = boost::shared_ptr< mGraph > (new mGraph ());

}

void Stub::cleanup(){
    mw_->toolbar_->removeAction(myaction);
    delete myaction;
}

Stub::~Stub(){
    qDebug() << "Stub deleted";
}

void Stub::myFunc(){
    qDebug() << "Myfunc";
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")