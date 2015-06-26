#include "plugins/camerasetter/camerasetter.h"
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
#include <QTextEdit>
#include <QTime>
#include <QTimer>
#include <QLineEdit>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

QString CameraSetter::getName(){
    return "CameraSetter";
}

void CameraSetter::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    is_enabled_ = false;
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    enable_ = new QAction(QIcon(":/camerasetter.png"), "Enable CameraSetter", 0);
    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);
    mw_->tooloptions_->addWidget(settings_);

    QTextEdit * camera_state_text = new QTextEdit();

    auto cameraToText = [this, camera_state_text](){
        auto rot = glwidget_->camera_.getRotation();
        auto pos = glwidget_->camera_.getPosition();
        QString str = QString("%1 %2 %3 \n %4 %5 %6 %7").arg(
                    QString::number(pos.x()),
                    QString::number(pos.y()),
                    QString::number(pos.z()),
                    QString::number(rot.x()),
                    QString::number(rot.y()),
                    QString::number(rot.z()),
                    QString::number(rot.w()));
        camera_state_text->setText(str);
    };

    auto textToCamera = [this, camera_state_text](){
        QString str = camera_state_text->toPlainText();

        QStringList sl = str.split(QRegExp("\\s+"), QString::SkipEmptyParts);
        if(sl.size() != 7){
            qDebug() << "No good";
            return;
        }

        QTextStream stream(&str);
        double rx, ry, rz, rw;
        double px, py, pz;

        stream >> px >> py >> pz >> rx >> ry >> rz >> rw;

        Eigen::Quaternion<float> rot(rw, rx, ry, rz);
        Eigen::Vector3f pos(px, py, pz);

        glwidget_->camera_.setRotation(rot);
        glwidget_->camera_.setPosition(pos);
    };

    connect(&glwidget_->camera_, &Camera::modified, cameraToText);
    connect(camera_state_text, &QTextEdit::textChanged, textToCamera);


    seconds_ = 0;
    running_ = false;
    target_ = Eigen::Vector3f(0, 0, 0);

    QLineEdit * elapsed_time_text = new QLineEdit();
    QTextEdit * target_text_area = new QTextEdit();
    target_text_area->setText("0, 0, 0");
    QPushButton * start_button = new QPushButton("Restart");

    connect(camera_state_text, &QTextEdit::textChanged, [this, camera_state_text](){
        QString str = camera_state_text->toPlainText();

        QStringList sl = str.split(QRegExp("\\s+"), QString::SkipEmptyParts);
        if(sl.size() != 3){
            qDebug() << "No good";
            return;
        }

        QTextStream stream(&str);
        double px, py, pz;

        stream >> px >> py >> pz;
        target_ = Eigen::Vector3f(pz, py, pz);
    });


    // Start when moving the camera
    connect(&glwidget_->camera_, &Camera::modified, [this, elapsed_time_text](){
        if(!running_){
            time_.restart();
            running_ = true;
        }


        Eigen::Vector3f zvector = glwidget_->camera_.getRotation() * Eigen::Vector3f::UnitZ();
        float rads = acos(zvector.dot(Eigen::Vector3f::UnitZ()));
        qDebug() << "rads" << rads;

        Eigen::Vector3f pos = glwidget_->camera_.getPosition();
        float dist = (pos - target_).norm();
        if(dist < 2 && fabs(rads - 1.57079) < 0.2 ){
            seconds_ = time_.elapsed()/1000;
            running_ = false;


            elapsed_time_text->setStyleSheet("QLineEdit { background: #C2DC5F;}");

            qDebug() << "done";
        } else {
            qDebug() << "dist" << dist;
        }
    });


    // Timer code

    timer_.connect(&timer_, &QTimer::timeout, [this, elapsed_time_text] () {
        if(!running_){
            //elapsed_time_text->setText("0");
        } else {
            seconds_ = time_.elapsed()/1000;
            elapsed_time_text->setStyleSheet("QLineEdit { background: #FFFFFF;}");
            elapsed_time_text->setText(QString::number(seconds_));
        }
    });
    timer_.start(1000);

    connect(start_button, &QPushButton::clicked, [this, elapsed_time_text](){
        elapsed_time_text->setText("0");
        elapsed_time_text->setStyleSheet("QLineEdit { background: #FFFFFF;}");
        seconds_ = 0;
        running_ = false;
    });

    layout->addWidget(new QLabel("Camera state"));
    layout->addWidget(camera_state_text);
    layout->addWidget(new QLabel("Target"));
    layout->addWidget(target_text_area);
    layout->addWidget(new QLabel("Time"));
    layout->addWidget(elapsed_time_text);
    layout->addWidget(start_button);
    layout->addStretch();
}

void CameraSetter::set() {
    qDebug() << "hello, this is where the action happens";
}

void CameraSetter::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    delete enable_;
}

CameraSetter::~CameraSetter(){
    
}

void CameraSetter::enable() {
    if(is_enabled_){
        disable();
        return;
    }

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);
    emit enabling();
    is_enabled_ = true;
}

void CameraSetter::disable(){
    enable_->setChecked(false);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.camerasetter")
