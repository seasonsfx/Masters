#include "plugins/snake/snake.h"
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
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "utilities/pointpicker.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

QString Snake::getName(){
    return "3D Brush Tool";
}

void Snake::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    enable_ = new QAction(QIcon(":/images/snake.png"), "2d snake lasso tool", 0);
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

void Snake::cleanup(){
    disconnect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    disconnect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    delete lasso_;
}

void Snake::paint(){
    lasso_->drawLasso(last_mouse_pos_.x(), last_mouse_pos_.y(), flatview_);
}


bool Snake::mouseClickEvent(QMouseEvent * event){
    lasso_->addPoint(event->x(), event->y(), core_->mw_->flatview_);
    return true;
}

bool Snake::mouseDblClickEvent(QMouseEvent * event){
    if(cl_->clouds_.size() == 0){
        disable();
        return false;
    }

    auto cloud = cl_->active_;

    std::shared_ptr<std::vector<int> > empty = std::make_shared<std::vector<int>>();
    std::shared_ptr<std::vector<int>> selected_indices = std::make_shared<std::vector<int>>();
    std::shared_ptr<std::vector<int>> removed_indices= std::make_shared<std::vector<int>>();


    // This is where the contour is modified

    // Distance between contour points should initially be less than D distance
    // Points need to be added on lines where this is not true
    // Assert points > 11 so the 4th order derivative works
    // Create an image to run the algorithm on
    // Down sampling might be required?
    // Use fuctions from utils.cpp
    // For each point calculate the internal and external energy
    // https://en.wikipedia.org/wiki/Active_contour_model
    // Calcuate x and y derivative seperately
    // Find appropriate weights
    //



/*
    lasso_->getIndices2D(cloud->scan_height(), flatview_->getCamera(),
                         cloud->cloudToGridMap(), selected_indices,
                         removed_indices);

    core_->us_->beginMacro("Snake lasso tool");
    core_->us_->push(new Select(cl_->active_, selected_indices, empty));
    core_->us_->endMacro();

    disable();
*/
    return true;
}

bool Snake::mouseMoveEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();

    if(cl_->clouds_.size() == 0) {
        disable();
        return false;
    }
    lasso_->movePoint(event->x(), event->y(), core_->mw_->flatview_);
    flatview_->update();

    if(event->buttons() != Qt::LeftButton)
        return false;

    flatview_->update();
    return true;
}

bool Snake::mousePressEvent(QMouseEvent * event) {
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

bool Snake::mouseReleaseEvent(QMouseEvent * event){
    last_mouse_pos_ << event->x(), event->y();
    float dist = (last_mouse_pos_ - mouse_down_pos_).norm();
    if(dist < 2){
        return mouseClickEvent(event);
    }

    return true;
}

void Snake::enable() {
    if(is_enabled_){
        disable();
        return;
    }
    QTabWidget * tabs = qobject_cast<QTabWidget *>(flatview_->parent()->parent());
    tabs->setCurrentWidget(flatview_);
    enable_->setChecked(true);

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);

    lasso_->clear();

    emit enabling();
    connect(flatview_, SIGNAL(pluginPaint()),
            this, SLOT(paint()),
            Qt::DirectConnection);
    flatview_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void Snake::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    disconnect(flatview_, SIGNAL(pluginPaint()),
            this, SLOT(paint()));
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

bool Snake::eventFilter(QObject *object, QEvent *event){

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
