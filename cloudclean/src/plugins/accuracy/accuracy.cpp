#include "plugins/accuracy/accuracy.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QWidget>
#include <QDockWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QListWidget>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QLineEdit>
#include <QLabel>
#include <QApplication>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

QString Accuracy::getName(){
    return "Accuracy";
}

void Accuracy::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    fscore_action_ = new QAction("F1", 0);
    fscore_action_->setShortcut(QKeySequence(Qt::Key_F1));

//    QStyle * style = QApplication::style();
//    fscore_action_->setIcon(style->standardIcon(QStyle::SP_CommandLink));

    mw_->toolbar_->addAction(fscore_action_);


    is_enabled_ = false;
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    enable_ = new QAction(QIcon(":/accuracy.png"), "Enable Accuracy", 0);
    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    layout->addWidget(new QLabel("Target:", settings_));
    QListWidget * l1 = new QListWidget(settings_);
    layout->addWidget(l1);
    QHBoxLayout * split1 = new QHBoxLayout(settings_);
    layout->addLayout(split1);
    QPushButton * add1 = new QPushButton("Add selected layers", settings_);
    split1->addWidget(add1);
    QPushButton * clear1 = new QPushButton("Clear", settings_);
    split1->addWidget(clear1);


    precision_ = new QLineEdit(settings_);
    precision_->setReadOnly(true);
    recall_ = new QLineEdit(settings_);
    recall_->setReadOnly(true);

    QHBoxLayout * split = new QHBoxLayout(settings_);
    layout->addLayout(split);
    split->addWidget(new QLabel("Recall", settings_));
    split->addWidget(recall_);
    split->addWidget(new QLabel("Precision", settings_));
    split->addWidget(precision_);

    // connect

    connect(add1, &QPushButton::clicked, [=] (){
        for(boost::weak_ptr<Layer> s : ll_->getSelection() ){
            bool found = false;
            for(boost::weak_ptr<Layer> e : target_){
                if(s.lock() == e.lock()){
                    found = true;
                    break;
                }
            }
            if(!found){
                target_.push_back(s);
                l1->addItem(s.lock()->getName());
            }
        }
        if(!target_.size()){
            fscore_action_->setVisible(true);
        }
    });


    connect(clear1, &QPushButton::clicked, [=] (){
        l1->clear();
        target_.clear();
        if(!target_.size()){
            fscore_action_->setVisible(false);
        }
    });

    connect(fscore_action_, SIGNAL(triggered()), this, SLOT(accuracy()));

    layout->addStretch();
}

void Accuracy::accuracy() {
    auto is_label_in_set = [=] (uint16_t label, std::vector<boost::weak_ptr<Layer> > & layers){
        const LayerSet & ls = ll_->getLayersForLabel(label);

        for(Layer * x : ls){
            for(boost::weak_ptr<Layer> y: layers){
                if(y.lock().get() == x)
                    return true;
            }
        }

        return false;
    };

    int true_positive_count = 0;
    int targetted_count = 0;
    int selected_count = 0;

    for(int i = 0; i < cl_->active_->points.size(); i++){
        uint16_t label = cl_->active_->labels_[i];
        bool selected = !!int8_t(cl_->active_->flags_[i]);
        bool targetted = is_label_in_set(label, target_);


        if(targetted){
            targetted_count++;
        }
        if(selected){
            selected_count++;
        }
        if(targetted && selected) {
            true_positive_count++;
        }
    }

    // http://en.wikipedia.org/wiki/Precision_and_recall
    qDebug() << "overlap: " << true_positive_count << "truth: " << targetted_count << "result: " << selected_count;
    qDebug() << "Recall: " << float(true_positive_count)/targetted_count;
    qDebug() << "Precision: " << float(true_positive_count)/selected_count;

    float recall = float(true_positive_count)/targetted_count;
    float precision = float(true_positive_count)/selected_count;
    float fscore = 2 * (precision * recall) / (precision + recall);

    recall_->setText(QString("%1").arg(recall));
    precision_->setText(QString("%1").arg(precision));
    fscore_action_->setText(QString("%1").arg(fscore));
}

void Accuracy::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    mw_->toolbar_->removeAction(fscore_action_);
    delete enable_;
}

Accuracy::~Accuracy(){
    
}

void Accuracy::enable() {
    qDebug() << "enabled?" << is_enabled_;
    if(is_enabled_){
        disable();
        return;
    }

    qDebug() << "enabled!";

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);
    emit enabling();
    glwidget_->installEventFilter(this);
    flatview_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void Accuracy::disable(){
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.accuracy")
