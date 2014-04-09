#include "plugins/autotest/autotest.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QWidget>
#include <QDockWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QSettings>
#include <QDir>
#include <QFileDialog>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "pluginsystem/pluginmanager.h"

#include "plugins/project/project.h"
#include "plugins/featureeval/featureeval.h"

QString AutoTest::getName(){
    return "AutoTest";
}

void AutoTest::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    is_enabled_ = false;
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    enable_ = new QAction(QIcon(":/autotest.png"), "Enable AutoTest", 0);
    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    QPushButton * open = new QPushButton("Open test");

    QPushButton * run = new QPushButton("Run test");
    run->setDisabled(true);

    connect(open, &QPushButton::clicked, [=] (){
        QSettings settings;

        QString path = settings.value("load/lasttestlocation", QDir::home().absolutePath()).toString();
        QString filename = QFileDialog::getOpenFileName(
                     nullptr, tr("Open test"), path , tr("JSON test files (*.json)"));
        if (filename.length() == 0)
            return;

        settings.setValue("load/lasttestlocation", QFileInfo(filename).absolutePath());
        settings.sync();
        run->setDisabled(false);
        test_path_ = filename;

    });

    layout->addWidget(open);
    layout->addWidget(run);

    connect(run, &QPushButton::clicked, (std::function<void()>) std::bind(&AutoTest::runtest, this));

    layout->addStretch();
}

void AutoTest::initialize2(PluginManager *pm){
    project_ = pm->findPlugin<Project>();
    feature_eval_ = pm->findPlugin<FeatureEval>();
}

void AutoTest::runtest() {
    // Setup reporting
    QFile report_file("report.txt");
    report_file.open(QIODevice::WriteOnly | QIODevice::Text);
    QDebug dbg(&report_file);
    feature_eval_->setReportFuction(&dbg);

    qDebug() << "Opening file: " << test_path_;
    QFile file(test_path_);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    if(!file.isOpen()){
        qDebug() << "Could not open file";
        return;
    }

    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &err);
    file.close();

    if(doc.isNull()){
        qDebug() << "No text data: " << err.errorString();
    }

    QJsonObject root = doc.object();
    QJsonObject features = root["features"].toObject();
    QJsonArray tests = root["tests"].toArray();

    for(QJsonValueRef test : tests){
        // setup project
        QJsonObject t = test.toObject();
        QString p = t["project"].toString();
        project_->load(p);
        ////////////////

        qDebug() << "Opening project: " << p << " ... ";

        QJsonArray correlations = t["correlations"].toArray();

        for(QJsonValueRef correlation : correlations){
            QJsonObject c = correlation.toObject();
            QString layer_name = c["layer_name"].toString();
            QJsonArray feature_names = c["features"].toArray();

            // Select layer
            int idx = ll_->getLayerIdxByName(layer_name);
            if(idx == -1){
                qDebug() << "Cannot find layer: " << layer_name;
                continue;
            }
            //std::vector<int> layers = {idx};
            ll_->selectionChanged({idx});


            for(QJsonValueRef feature_name : feature_names){
                QString fname = feature_name.toString();
                QJsonObject params = features[fname].toObject();



                qDebug() << "Correlating " << fname << " with layer " << layer_name;


                for(QJsonObject::Iterator it = params.begin(); it != params.end(); it++){
                    QString param_name = it.key();
                    for(QJsonValueRef valref : it.value().toArray()){
                        float val = valref.toDouble();
                        qDebug() << "Parameter " << param_name << " set to: " << val;
                    }
                    //param.toObject()
                }

                //QString fname = feature["name"].toString();

                // Get layer by name
                // activate layer

                // set parameter permutations

                // resolve function call:
                feature_eval_->getFunction(fname)();

            }


        }




        // clean up project
        while(ll_->rowCount() > 0){
            ll_->deleteLayer(0);
        }

        while(cl_->rowCount() > 0){
            cl_->removeCloud(0);
        }
        //////////////////////
        report_file.close();
    }
    //features;


}

void AutoTest::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    delete enable_;
}

AutoTest::~AutoTest(){
    
}

void AutoTest::enable() {
    if(is_enabled_){
        disable();
        return;
    }

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);
    emit enabling();
    glwidget_->installEventFilter(this);
    flatview_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void AutoTest::disable(){
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.autotest")
