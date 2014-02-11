#include "plugins/brush2d/brush2d.h"
#include <QDebug>
#include <QEvent>
#include <QKeyEvent>
#include <QAction>
#include <QTabWidget>
#include <QApplication>
#include <QToolBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QSpacerItem>
#include <QStackedWidget>
#include <QSlider>
#include <QDockWidget>
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

QString Brush2D::getName(){
    return "2D Brush Tool";
}

void Brush2D::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;
    us_ = core->us_;

    select_mask_ = 1;

    enable_ = new QAction(QIcon(":/images/brush.png"), "2d Brush Tool", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);

    is_enabled_ = false;

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));

    mw_->addMenu(enable_, "Edit");
    mw_->toolbar_->addAction(enable_);

    radius_ = 20;

    settings_ = new QWidget();
    settings_->setLayout(new QHBoxLayout(settings_));

    QSlider * slider = new QSlider(settings_);
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(1, 1000);
    slider->setSingleStep(1);
    slider->setToolTip("Radius in pixels");
    slider->setValue(radius_);
    slider->setTickPosition(QSlider::TicksBelow);

    QLabel * label = new QLabel("Radius", settings_);

    settings_->layout()->addWidget(label);
    settings_->layout()->addWidget(slider);

    mw_->tooloptions_->addWidget(settings_);

    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(setRad(int)));

}

void Brush2D::setRad(int val) {
    radius_ = val;
}

void Brush2D::cleanup(){
    mw_->removeMenu(enable_, "Edit");
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    disconnect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    disconnect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
}

void Brush2D::select(QMouseEvent * event){

    bool negative_select = QApplication::keyboardModifiers() == Qt::ControlModifier;

    boost::shared_ptr<PointCloud> pc = cl_->active_;

    Eigen::Vector3f coord;
    coord << 2.0f* (event->x()/float(flatview_->width()) - 0.5),
        -2.0f* (event->y()/float(flatview_->height()) - 0.5), 1;
    coord = flatview_->getCamera().inverse() * coord;

    coord[1] = cl_->active_->scan_height()-coord[1];

    boost::shared_ptr<std::vector<int> > indices;
    indices.reset(new std::vector<int>());

    for(int x = -radius_/2; x < radius_/2; x++){
        for(int y = -radius_/2; y < radius_/2; y++){
            if(x*x + y*y > (radius_/2.0f)*(radius_/2.0f) )
                continue;

            int idx = flatview_->imageToCloudIdx(int(coord.x() + x + 0.5),
                                      int(coord.y() + y + 0.5), pc);
            if (idx < 0) {
                if(idx < -1)
                    qDebug() << "Bug! Idx < -1 : idx = " << idx;
                continue;
            }

            indices->push_back(idx);

        }
    }

    us_->push(new Select(pc, indices, negative_select, select_mask_));

}


bool Brush2D::mouseMoveEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();
    if(event->buttons() != Qt::LeftButton)
        return false;
    if(cl_->active_.get() == nullptr)
        return false;

    select(event);
    return true;
}

bool Brush2D::mousePressEvent(QMouseEvent * event) {
    if(event->buttons() != Qt::LeftButton)
        return false;
    if(cl_->active_.get() == nullptr)
        return false;

    core_->us_->beginMacro("2d Select");
    select(event);
    last_mouse_pos_ << event->x(), event->y();
    mouse_down_pos_ = last_mouse_pos_;
    return true;
}

bool Brush2D::mouseReleaseEvent(QMouseEvent * event){
    core_->us_->endMacro();
    last_mouse_pos_ << event->x(), event->y();

    return true;
}

bool Brush2D::mouseWheelEvent(QWheelEvent * event){
    int delta = event->delta()/120;

    bool valid_upscale = delta > 0 && radius_ < cl_->active_->scan_height() &&
            radius_ < cl_->active_->scan_width();

    bool valid_downscale = delta < 0 && radius_ > 1;

    if(valid_upscale || valid_downscale){
        radius_ += delta;
    }
    return true;
}

void Brush2D::enable() {
    if(is_enabled_){
        disable();
        return;
    }
    mw_->tooloptions_->setCurrentWidget(settings_);
    mw_->options_dock_->show();
    QTabWidget * tabs = qobject_cast<QTabWidget *>(flatview_->parent()->parent());
    tabs->setCurrentWidget(flatview_);
    enable_->setChecked(true);
    emit enabling();
    flatview_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void Brush2D::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

bool Brush2D::eventFilter(QObject *object, QEvent *event){
    // Bypass plugin via shift
    if(QApplication::keyboardModifiers() == Qt::SHIFT)
        return false;

    switch(event->type()){
    case QEvent::MouseButtonPress:
        return mousePressEvent(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
        return mouseReleaseEvent(static_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
        return mouseMoveEvent(static_cast<QMouseEvent*>(event));
    case QEvent::KeyPress:
        if(static_cast<QKeyEvent*>(event)->key() == Qt::Key_Control)
            return true;

        switch(static_cast<QKeyEvent*>(event)->key()){
        case Qt::Key_1:
            select_mask_ = 1;
            return true;
        case Qt::Key_2:
            select_mask_ = 2;
            return true;
        case Qt::Key_3:
            select_mask_ = 4;
            return true;
        case Qt::Key_4:
            select_mask_ = 8;
            return true;
        case Qt::Key_5:
            select_mask_ = 16;
            return true;
        case Qt::Key_6:
            select_mask_ = 32;
            return true;
        case Qt::Key_7:
            select_mask_ = 64;
            return true;
        case Qt::Key_8:
            select_mask_ = 128;
            return true;
        }
    //case QEvent::Wheel:
    //    return mouseWheelEvent(static_cast<QWheelEvent*>(event));
    default:
        return false;
    }
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
