#include "plugins/jsonexport/jsonexport.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <fstream>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

#include <pcl/search/kdtree.h>

QString JsonExport::getName(){
    return "JsonExport";
}

void JsonExport::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    myaction = new QAction(QIcon(":/jsonexport.png"), "JsonExport action", 0);

    connect(myaction,&QAction::triggered, [this] (bool on) {
        qDebug() << "Click!";
    });

    connect(myaction, SIGNAL(triggered()), this, SLOT(myFunc()));

    mw_->toolbar_->addAction(myaction);
}

void JsonExport::cleanup(){
    mw_->toolbar_->removeAction(myaction);
    delete myaction;
}

JsonExport::~JsonExport(){
    qDebug() << "JsonExport deleted";
}

void JsonExport::myFunc(){
    qDebug() << "Myfunc";


//    filecount;
//    filename1;
//    filename2;
//    file1 label count;
//    1 2 3 4 5 5 6 6 9 9
//    file2 label count;
//    667 2 26237 4724 27
//    layer count;
//    layer1 name;
//    layer1 visibility;
//    layer1 label count;
//    3 324 242 423 42 34234
//    EOF


    // Bring up file save dialog here

    std::ofstream file("test.ccf");
    if (!file.is_open()){
        qDebug() << "File open fail";
        return;
    }

    file << cl_->clouds_.size() << "\n";

    for(boost::shared_ptr<PointCloud> cloud :cl_->clouds_)
        file << cloud->filepath().data()->toLatin1() << "\n";

    for(boost::shared_ptr<PointCloud> cloud :cl_->clouds_){
        file << cloud->labels_.size() << "\n";
        int idx = 0;
        for(; idx < cloud->labels_.size()-1; idx++){
            file << cloud->labels_[idx] << " ";
        }
        file << cloud->labels_[++idx] << "\n";
    }

    file << ll_->getLayers().size() << "\n";

    for(const boost::shared_ptr<Layer> layer : ll_->getLayers()) {

        file << layer->getName().data()->toLatin1() << "\n";
        file << layer->isVisible() << "\n";
        file << layer->getLabelSet().size() << "\n";

        const std::set<uint16_t> & ls = layer->getLabelSet();
        auto it = ls.begin();
        for(; it != --ls.end(); it++) {
            file << *it << " ";
        }
        file << *it << "\n";
    }

    file.close();
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.jsonexport")
