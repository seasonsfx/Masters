#include "plugins/stub/stub.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QtWidgets>

#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "utilities/pointpicker.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

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
    connect(myaction, SIGNAL(triggered()), core_, SIGNAL(endEdit()));

    mw_->toolbar_->addAction(myaction);

    settings_ = new QWidget();
    QGridLayout * layout = new QGridLayout(settings_);
    settings_->setLayout(layout);
    mw_->tooloptions_->addWidget(settings_);

    QSpinBox * kconnect = new QSpinBox(settings_);
    QDoubleSpinBox * sigma = new QDoubleSpinBox(settings_);
    QDoubleSpinBox * source_weight = new QDoubleSpinBox(settings_);
    QDoubleSpinBox * radius = new QDoubleSpinBox(settings_);

    k_connect_ = 14;
    sigma_ = 0.25;
    source_weight_ = 0.80;
    radius_ = 3;

    kconnect->setValue(k_connect_);
    sigma->setValue(sigma_);
    source_weight->setValue(source_weight_);
    radius->setValue(radius_);

    QLabel * label_kcon = new QLabel("K connectivity", settings_);
    QLabel * label_sigma = new QLabel("Sigma (Density)", settings_);
    QLabel * label_source_w = new QLabel("Source weight", settings_);
    QLabel * label_radius = new QLabel("Radius (meters)", settings_);

    layout->addWidget(label_kcon);
    layout->addWidget(kconnect);

    layout->addWidget(label_sigma);
    layout->addWidget(sigma);

    layout->addWidget(label_source_w);
    layout->addWidget(source_weight);

    layout->addWidget(label_radius);
    layout->addWidget(radius);

    connect(kconnect, (void (QSpinBox:: *)(int)) &QSpinBox::valueChanged, [this] (int val) {
        k_connect_ = val;
    });

    connect(sigma, (void (QDoubleSpinBox:: *)(double)) &QDoubleSpinBox::valueChanged, [this] (double val) {
        sigma_ = val;
    });

    connect(source_weight, (void (QDoubleSpinBox:: *)(double)) &QDoubleSpinBox::valueChanged, [this] (double val) {
        radius_ = val;
    });

    connect(radius, (void (QDoubleSpinBox:: *)(double)) &QDoubleSpinBox::valueChanged, [this] (double val) {
        radius_ = val;
    });

}

void Stub::cleanup(){
    mw_->toolbar_->removeAction(myaction);
    delete myaction;
}

Stub::~Stub(){
    qDebug() << "Stub deleted";
}

void Stub::myFunc(){
    qDebug() << "Myfunc!!";
}

bool Stub::eventFilter(QObject *object, QEvent *event){

    // Bypass plugin via shift
    if(QApplication::keyboardModifiers() == Qt::SHIFT)
        return false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
