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
    initialized_gl = false;

    enable_ = new QAction(QIcon(":/images/lasso.png"), "3d polygon lasso tool", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);

    is_enabled_ = false;
    radius_ = 0.5f;

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));

    mw_->addMenu(enable_, "Edit");
    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    QHBoxLayout * hb = new QHBoxLayout(settings_);
    layout->addItem(hb);
    layout->addItem(new QSpacerItem(0, 0, QSizePolicy::Maximum, QSizePolicy::Maximum));


    QSlider * slider = new QSlider(settings_);
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(1, 300);
    slider->setSingleStep(1);
    slider->setToolTip("Radius in cm");
    slider->setValue(radius_*100);
    slider->setTickPosition(QSlider::TicksBelow);

    QLabel * label = new QLabel("Radius", settings_);

    hb->addWidget(label);
    hb->addWidget(slider);

    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(setRad(int)));

    lasso_ = new Lasso();
}

void Lasso3D::setRad(int val) {
    radius_ = val/100.0;
}

void Lasso3D::cleanup(){
    disconnect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    disconnect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    delete line_;
    delete program_;
    delete lasso_;
}

void Lasso3D::initializeGL() {
    /*
    program_ = new QGLShaderProgram();
    bool succ = program_->addShaderFromSourceFile(
                QGLShader::Vertex, ":/basic.vert"); CE();
    qWarning() << program_->log();
    if (!succ) qWarning() << "Shader compile log:" << program_->log();
    succ = program_->addShaderFromSourceFile(
                QGLShader::Fragment, ":/basic.frag"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_->log();
    succ = program_->link(); CE();
    if (!succ) {
        qWarning() << "Could not link shader program_:" << program_->log();
        qWarning() << "Exiting...";
        abort();
    }

    line_ = new QGLBuffer();
    line_->create(); CE();
    line_->bind(); CE();
    size_t point_size = 3*sizeof(float);
    line_->allocate(2*point_size); CE();
    line_->release();

    initialized_gl = true;
    */
}

void Lasso3D::paint(const Eigen::Affine3f& proj, const Eigen::Affine3f& mv){
    /*
    if(!initialized_gl) {
        initializeGL();
    }
    */

    lasso_->drawLasso(last_mouse_pos_.x(), last_mouse_pos_.y(), glwidget_);

    /*
    program_->bind();

    line_->bind(); CE();
    float * layerbuff =
            static_cast<float *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();

    layerbuff[0] = p1.x();
    layerbuff[1] = p1.y();
    layerbuff[2] = p1.z();
    layerbuff[3] = p2.x();
    layerbuff[4] = p2.y();
    layerbuff[5] = p2.z();

    glUnmapBuffer(GL_ARRAY_BUFFER);

    glUniformMatrix4fv(program_->uniformLocation("mv"), 1, GL_FALSE,
                       glwidget_->camera_.modelviewMatrix().data());CE();
    glUniformMatrix4fv(program_->uniformLocation("proj"), 1, GL_FALSE,
                       glwidget_->camera_.projectionMatrix().data()); CE();
    float col[3] = {0, 1, 0};
    glUniform3fv(program_->uniformLocation("colour"), 1, col); CE();

    glEnableVertexAttribArray(0); CE();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0); CE();

    glLineWidth(2);
    glDrawArrays(GL_LINES, 0, 2); CE();

    line_->release(); CE();

    program_->release();
    */
}

void Lasso3D::select(QMouseEvent * event){
    int idx = pick(event->x(), event->y(), glwidget_->width(),
                   glwidget_->height(), 1e-04,
                   glwidget_->camera_.projectionMatrix(),
                   glwidget_->camera_.modelviewMatrix(),
                   cl_->active_);

    if(idx == -1)
        return;

    std::shared_ptr<std::vector<int> > indices;
    indices.reset(new std::vector<int>());

    std::shared_ptr<std::vector<int> > empty;
    empty.reset(new std::vector<int>());

    std::vector<float> distsq;
    cl_->active_->getOctree()->radiusSearch(idx, radius_, *indices, distsq);

    bool negative_select = QApplication::keyboardModifiers() == Qt::ControlModifier;

    if(negative_select)
        core_->us_->push(new Select(cl_->active_, empty, indices));
    else
        core_->us_->push(new Select(cl_->active_, indices, empty));

}

bool Lasso3D::mouseClickEvent(QMouseEvent * event){
    lasso_->addPoint(event->x(), event->y(), core_->mw_->glwidget_);
    return true;
}

bool Lasso3D::mouseDblClickEvent(QMouseEvent * event){
    if(cl_->clouds_.size() == 0){
        disable();
        return false;
    }

    auto cam = core_->mw_->glwidget_->camera_;
    auto cloud = cl_->active_;

    Eigen::Matrix4f ndc = (cam.projectionMatrix() *  cam.modelviewMatrix() * cloud->modelview()).matrix();

    std::shared_ptr<std::vector<int>> source_indices;
    std::shared_ptr<std::vector<int>> removed_indices;
    lasso_->getIndices(ndc, cloud.get(), source_indices, removed_indices);
    disable();
    return true;
}

bool Lasso3D::mouseMoveEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();
    if(event->buttons() != Qt::LeftButton)
        return false;
    if(cl_->clouds_.size() == 0) {
        disable();
        return false;
    }

    if(event->buttons())
        select(event);

    return true;
}

bool Lasso3D::mousePressEvent(QMouseEvent * event) {
    if(event->buttons() != Qt::LeftButton)
        return false;
    if(cl_->clouds_.size() == 0){
        disable();
        return false;
    }
    core_->us_->beginMacro("3d Select");
    select(event);
    last_mouse_pos_ << event->x(), event->y();
    mouse_down_pos_ = last_mouse_pos_;
    return true;
}

bool Lasso3D::mouseReleaseEvent(QMouseEvent * event){
    core_->us_->endMacro();
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
