#include "plugins/jsonexport/jsonexport.h"
#include <QDebug>
#include <QAction>
#include <QFileDialog>
#include <QToolBar>
#include <QStyle>
#include <QApplication>
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

    QStyle * style = QApplication::style();
    save_action_ = new QAction(style->standardIcon(QStyle::SP_DialogSaveButton),"Save project", 0);
    load_action_ = new QAction(style->standardIcon(QStyle::SP_DirIcon), "Load project", 0);



    connect(save_action_,&QAction::triggered, [this] (bool on) {
        qDebug() << "Click!";
    });

    connect(save_action_, SIGNAL(triggered()), this, SLOT(save()));
    connect(load_action_, SIGNAL(triggered()), this, SLOT(load()));


    mw_->addMenu(save_action_, "File");
    mw_->addMenu(load_action_, "File");
    //mw_->toolbar_->addAction(save_action_);
    //mw_->toolbar_->addAction(load_action_);
}

void JsonExport::cleanup(){
    mw_->removeMenu(save_action_, "File");
    mw_->removeMenu(load_action_, "File");
    //mw_->toolbar_->removeAction(save_action_);
    //mw_->toolbar_->removeAction(load_action_);
    delete save_action_;
    delete load_action_;
}

JsonExport::~JsonExport(){
    qDebug() << "JsonExport deleted";
}

void JsonExport::save(){
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


    QString filename = QFileDialog::getSaveFileName(
                 nullptr, tr("Save project"), QDir::home().absolutePath(), tr("Cloud clean project files (*.ccp)"));
    if (filename.length() == 0)
        return;

    std::ofstream file(filename.toLocal8Bit().data());
    if (!file.is_open()){
        qDebug() << "File open fail";
        return;
    }

    file << cl_->clouds_.size() << "\n";

    for(boost::shared_ptr<PointCloud> cloud :cl_->clouds_) {
        file << cloud->filepath().toLocal8Bit().data() << "\n";
        qDebug() << "Saving: " << cloud->filepath();
    }

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

        file << layer->getName().toLocal8Bit().data() << "\n";
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

void JsonExport::load(){
    QString filename = QFileDialog::getOpenFileName(
                 nullptr, tr("Open project"), QDir::home().absolutePath(), tr("Cloud clean project files (*.ccp)"));
    if (filename.length() == 0)
        return;

    std::ifstream file(filename.toLocal8Bit().data());
    if (!file.is_open()){
        qDebug() << "File open fail";
        return;
    }


    int num_clouds;
    file >> num_clouds >> std::ws;
    qDebug() << "Number of clouds" << num_clouds;
    std::vector<QString> filepaths(num_clouds);

    char buff[1024];

    for(QString & path : filepaths) {
        file.getline(buff, 1024);
        path = QString::fromLocal8Bit(buff);
        qDebug() << "cloud path" << path;
    }

    std::map<uint16_t, uint16_t> old_to_new_label;

    for(QString path : filepaths) {
        boost::shared_ptr<PointCloud> cloud = cl_->loadFile(path);
        if(cloud == nullptr) {
            qDebug() << "could not load cloud";
            return;
        }
        std::vector<int16_t> & labels = cloud->labels_;
        int labelcount;
        int label;
        file >> labelcount >> std::ws;
        qDebug() << "Label count: " << labelcount;
        for(int i = 0; i < labelcount; i++) {
            file >> label;
            if(old_to_new_label.find(label) == old_to_new_label.end())
                old_to_new_label[label] = ll_->createLabelId();
            labels[i] = old_to_new_label[label];
        }


    }

    int layercount;
    file >> layercount >> std::ws;

    for(int i = 0; i < layercount; i++) {

        file.getline(buff, 1024);
        QString name(buff);
        boost::shared_ptr<Layer> layer = ll_->addLayer(name);

        bool visible;
        file >> visible;
        if(!visible)
            layer->toggleVisible();


        int label_count;
        file >> label_count;

        uint16_t label;
        for(int j = 0; j < label_count; j++){
            file >> label >> std::ws;
            layer->addLabel(old_to_new_label[label]);
        }

        emit ll_->layerUpdate(layer);
    }

    file.close();

    emit ll_->lookupTableUpdate();

    //std::thread(&CloudList::loadFile, cl_, filename).detach();
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.jsonexport")