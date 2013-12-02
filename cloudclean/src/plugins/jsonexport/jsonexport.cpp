#include "plugins/jsonexport/jsonexport.h"
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QAction>
#include <QToolBar>
#include <QFile>
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

// Example
//    {
//        "filenames": ["one.ptx", "two.ptx"],
//        "labels": {
//            "one.ptx": [1, 2, 3, 4, 5],
//            "two.ptx": [1, 2, 3, 4, 5],
//        },
//        "layers": [
//            {
//                "name": "Layer1",
//                "visible": true,
//                "labels": [1, 3, 5],
//            }
//        ]
//    }



    QJsonObject root;
    QJsonArray filenames;
    QJsonObject labels;

    for(boost::shared_ptr<PointCloud> cloud :cl_->clouds_){
        filenames.push_back(QJsonValue(cloud->filepath()));

        QJsonArray pointlabels;

        for(int label : cloud->labels_){
            pointlabels.push_back(QJsonValue(label));
        }

        labels.insert(cloud->filepath(), QJsonValue(pointlabels));
    }

    root.insert("filenames", QJsonValue(filenames));
    root.insert("labels", QJsonValue(labels));

    QJsonArray layers;

    for(const boost::shared_ptr<Layer> layer : ll_->getLayers()) {
        QJsonObject alayer;
        alayer.insert("name", QJsonValue(layer->getName()));
        alayer.insert("visible", QJsonValue(layer->isVisible()));
        QJsonArray labels;
        for(int label : layer->getLabelSet()) {
            labels.push_back(QJsonValue(label));
        }
        alayer.insert("labels", QJsonValue(labels));
        layers.push_back(alayer);
    }

    root.insert("layers", QJsonValue(layers));

    QJsonDocument doc(root);

    QFile file;
    file.setFileName("test.ccf");
    file.open(QIODevice::WriteOnly);
    file.write(doc.toJson());
    file.close();


    // Bring up file save dialog here

}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.jsonexport")
