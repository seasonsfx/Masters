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
#include <QApplication>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "utilities/pointpicker.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "utilities/cv.h"
#include "plugins/snake/algo.h"

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
    min_segment_len_ = 50;

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
    if(!done_)
        lasso_->addScreenPoint(event->x(), event->y(), core_->mw_->flatview_->width(), core_->mw_->flatview_->height());
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


    std::vector<Eigen::Vector2f> points = lasso_->getPoints();

    Lasso * new_lasso = new Lasso();

    for(uint idx1 = 0; idx1 < points.size(); idx1++) {
       new_lasso->addNormPoint(points[idx1]);

       int idx2 = (idx1 + 1) % points.size();

       Eigen::Vector2i p1 = Lasso::getScreenPoint(points[idx1],
                                     flatview_->width(), flatview_->height());
       Eigen::Vector2i p2 = Lasso::getScreenPoint(points[idx2],
                                     flatview_->width(), flatview_->height());
       float dist = (p2-p1).cast<float>().norm();

       if(dist <= min_segment_len_)
           continue;

       Eigen::Vector2f dir = (p2-p1).cast<float>().normalized();

       float dist_along_segment = 0;

       while (dist_along_segment+min_segment_len_ < dist) {
            dist_along_segment += min_segment_len_;
            Eigen::Vector2f new_point = dist_along_segment * dir + p1.cast<float>();
            new_lasso->addScreenPoint(new_point.x(), new_point.y(), flatview_->width(), flatview_->height());
       }


    }

    delete lasso_;
    lasso_ = new_lasso;

    points = lasso_->getPoints();

    int h = cloud->scan_width();
    int w = cloud->scan_height();

    // Create distance map
    std::shared_ptr<std::vector<float> > distmap = makeDistmap(cloud);
    std::shared_ptr<std::vector<float> > grad_image = gradientImage(distmap, w, h);
    std::shared_ptr<std::vector<float> > smooth_grad_image = convolve(grad_image, w, h, gaussian, 5);

    // problem!
    // Points are in ndc space

    std::vector<Eigen::Vector2i> spoints;

    qDebug() << "New size" << points.size();

    for(Eigen::Vector2f & p : points) {
        Eigen::Vector2i sp = Lasso::getScreenPoint(p, w, h);
        qDebug() << "Point:" << sp.x() << sp.y();
        spoints.push_back(sp);
    }

    for(Eigen::Vector2i & p : spoints) {
        qDebug() << "Out: " << p.x() << p.y();
    }

    int it = 0;
    bool converged = false;
    while(!converged && it++ < 100) {

        converged = snake_iteration(smooth_grad_image,
                      w,
                      h,
                      spoints,
                      11,
                      11,
                      1,
                      1,
                      1);


        new_lasso = new Lasso();

        for(Eigen::Vector2i & p : spoints) {
            //qDebug() << p.x() << p.y();
            new_lasso->addScreenPoint(p.x(), p.y(), w, h);
        }

        delete lasso_;
        lasso_ = new_lasso;

        qDebug() << it;

        flatview_->update(0, 0 , flatview_->width(), flatview_->height());
        QApplication::processEvents();
    }


   // points are now control points

   // for each point around the control groups, calculate the cost function,
   // move to the lowest?


/*
    lasso_->getIndices2D(cloud->scan_height(), flatview_->getCamera(),
                         cloud->cloudToGridMap(), selected_indices,
                         removed_indices);

    core_->us_->beginMacro("Snake lasso tool");
    core_->us_->push(new Select(cl_->active_, selected_indices, empty));
    core_->us_->endMacro();

    disable();
*/
    done_ = true;
    return true;
}

bool Snake::mouseMoveEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();

    if(cl_->clouds_.size() == 0) {
        disable();
        return false;
    }

    if(!done_)
        lasso_->moveLastScreenPoint(event->x(), event->y(), core_->mw_->flatview_);
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
    done_ = false;
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
