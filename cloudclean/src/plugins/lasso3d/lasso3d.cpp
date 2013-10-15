#include "plugins/lasso3d/lasso3d.h"
#include <QDebug>
#include <QEvent>
#include <QKeyEvent>
#include <QAction>
#include <QGLShaderProgram>
#include <QGLBuffer>
#include <QTabWidget>
#include <QApplication>
#include <QToolBar>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QSpacerItem>
#include <QStackedWidget>
#include <QSlider>
#include <QDockWidget>
#include <boost/make_shared.hpp>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "utilities/pointpicker.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

QString Lasso3D::getName(){
    return "3D Brush Tool";
}

void Lasso3D::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    enable_ = new QAction(QIcon(":/images/lasso_3d.png"), "3d polygon lasso tool", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);

    is_enabled_ = false;

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));

    mw_->addMenu(enable_, "Edit");
    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    lasso_ = new Lasso();
}

void Lasso3D::cleanup(){
    disconnect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    disconnect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    delete lasso_;
}

void Lasso3D::paint(const Eigen::Affine3f& proj, const Eigen::Affine3f& mv){
    lasso_->drawLasso(last_mouse_pos_.x(), last_mouse_pos_.y(), glwidget_);
}


bool Lasso3D::mouseClickEvent(QMouseEvent * event){
    lasso_->addScreenPoint(event->x(), event->y(), core_->mw_->glwidget_->width(), core_->mw_->glwidget_->height());
    return true;
}

bool Lasso3D::mouseDblClickEvent(QMouseEvent * event){
    if(cl_->clouds_.size() == 0){
        disable();
        return false;
    }

    auto & cam = core_->mw_->glwidget_->camera_;
    auto cloud = cl_->active_;

    Eigen::Matrix4f ndc = (cam.projectionMatrix() *  cam.modelviewMatrix() * cloud->modelview()).matrix();

    boost::shared_ptr<std::vector<int> > empty = boost::make_shared<std::vector<int>>();
    boost::shared_ptr<std::vector<int>> selected_indices = boost::make_shared<std::vector<int>>();
    boost::shared_ptr<std::vector<int>> removed_indices= boost::make_shared<std::vector<int>>();

    lasso_->getIndices(ndc, cloud.get(), selected_indices, removed_indices);

    core_->us_->beginMacro("3d lasso tool");
    core_->us_->push(new Select(cl_->active_, selected_indices, empty));
    core_->us_->endMacro();

    disable();
    return true;
}

bool Lasso3D::mouseMoveEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();

    if(cl_->clouds_.size() == 0) {
        disable();
        return false;
    }
    lasso_->moveLastScreenPoint(event->x(), event->y(), core_->mw_->glwidget_);
    glwidget_->update();

    if(event->buttons() != Qt::LeftButton)
        return false;

    glwidget_->update();
    return true;
}

bool Lasso3D::mousePressEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();
    mouse_down_pos_ = last_mouse_pos_;
    if(event->buttons() != Qt::LeftButton)
        return false;
    if(cl_->clouds_.size() == 0){
        disable();
        return false;
    }

    return true;
}

bool Lasso3D::mouseReleaseEvent(QMouseEvent * event){
    last_mouse_pos_ << event->x(), event->y();
    float dist = (last_mouse_pos_ - mouse_down_pos_).norm();
    if(dist < 2){
        return mouseClickEvent(event);
    }

    return true;
}

void Lasso3D::enable() {
    if(is_enabled_){
        disable();
        return;
    }
    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
    tabs->setCurrentWidget(glwidget_);
    enable_->setChecked(true);

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);

    lasso_->clear();

    emit enabling();
    connect(glwidget_, SIGNAL(pluginPaint(Eigen::Affine3f, Eigen::Affine3f)),
            this, SLOT(paint(Eigen::Affine3f, Eigen::Affine3f)),
            Qt::DirectConnection);
    glwidget_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void Lasso3D::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    disconnect(glwidget_, SIGNAL(pluginPaint(Eigen::Affine3f, Eigen::Affine3f)),
            this, SLOT(paint(Eigen::Affine3f, Eigen::Affine3f)));
    glwidget_->removeEventFilter(this);
    is_enabled_ = false;
}

bool Lasso3D::eventFilter(QObject *object, QEvent *event){

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
    case QEvent::MouseButtonDblClick:
        return mouseDblClickEvent(static_cast<QMouseEvent*>(event));
    default:
        return false;
    }
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
