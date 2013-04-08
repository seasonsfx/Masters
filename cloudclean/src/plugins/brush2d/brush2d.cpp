#include "plugins/brush2d/brush2d.h"
#include <QDebug>
#include <QEvent>
#include <QKeyEvent>
#include <QAction>
#include <QTabWidget>
#include <QApplication>
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

    enable_ = new QAction("2d Brush Tool", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);

    is_enabled_ = false;

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));

    mw_->addMenu(enable_, "Edit");

    radius = 20;
}

void Brush2D::cleanup(){
    disconnect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    disconnect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
}

void Brush2D::select(QMouseEvent * event){

    bool negative_select = QApplication::keyboardModifiers() == Qt::ControlModifier;

    std::shared_ptr<PointCloud> pc = cl_->active_;

    Eigen::Vector3f coord;
    coord << 2.0f* (event->x()/float(flatview_->width()) - 0.5),
        -2.0f* (event->y()/float(flatview_->height()) - 0.5), 1;
    coord = flatview_->getCamera().inverse() * coord;

    coord[1] = cl_->active_->scan_height_-coord[1];

    std::shared_ptr<std::vector<int> > selected;
    selected.reset(new std::vector<int>());

    std::shared_ptr<std::vector<int> > deselected;
    deselected.reset(new std::vector<int>());

    for(int x = -radius/2; x < radius/2; x++){
        for(int y = -radius/2; y < radius/2; y++){
            if(x*x + y*y > (radius/2.0f)*(radius/2.0f) )
                continue;

            int idx = flatview_->imageToCloudIdx(int(coord.x() + x + 0.5),
                                      int(coord.y() + y + 0.5));
            if (idx != -1){
                if(negative_select)
                    deselected->push_back(idx);
                else
                    selected->push_back(idx);
            }
        }
    }

    us_->push(new Select(pc, selected, deselected));

}


bool Brush2D::mouseMoveEvent(QMouseEvent * event) {
    if(event->buttons())
        select(event);
    last_mouse_pos_ << event->x(), event->y();
    return true;
}

bool Brush2D::mousePressEvent(QMouseEvent * event) {
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

    bool valid_upscale = delta > 0 && radius < cl_->active_->scan_height_ &&
            radius < cl_->active_->scan_width_;

    bool valid_downscale = delta < 0 && radius > 1;

    if(valid_upscale || valid_downscale){
        radius += delta;
    }
    return true;
}

void Brush2D::enable() {
    if(is_enabled_){
        disable();
        return;
    }
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
    case QEvent::Wheel:
        return mouseWheelEvent(static_cast<QWheelEvent*>(event));
    default:
        return false;
    }
}

Q_EXPORT_PLUGIN2(pnp_brush2d, Brush2D)