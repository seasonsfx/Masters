#include "plugins/brush3d/brush3d.h"
#include <QDebug>
#include <QEvent>
#include <QKeyEvent>
#include <QAction>
#include <QGLShaderProgram>
#include <QGLBuffer>
#include <QTabWidget>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "actionmanager.h"
#include "utilities/pointpicker.h"

QString Brush3D::getName(){
    return "3D Brush Tool";
}

void Brush3D::initialize(PluginManager * pm, ActionManager *am, CloudList * cl, LayerList * ll,
                         GLWidget * glwidget, FlatView * flatview){
    cl_ = cl;
    ll_ = ll;
    glwidget_ = glwidget;
    flatview_ = flatview;
    am_ = am;
    pm_ = pm;
    initialized_gl = false;

    enable_ = new QAction("3d Brush Tool", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);

    is_enabled_ = false;

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    connect(this, SIGNAL(enabling()), (QObject *)pm_, SIGNAL(endEdit()));

    am_->addAction(enable_, "Edit");
}

void Brush3D::cleanup(){
    disconnect(this, SIGNAL(enabling()), (QObject *)pm_, SIGNAL(endEdit()));
    disconnect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    delete line_;
    delete program_;
}

void Brush3D::initializeGL() {
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
}

void Brush3D::paint(Eigen::Affine3f, Eigen::Affine3f){
    if(!initialized_gl) {
        initializeGL();
    }

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
    glVertexAttribPointer(0, 3, GL_FLOAT, FALSE, 0, 0); CE();

    glLineWidth(2);
    glDrawArrays(GL_LINES, 0, 2); CE();

    line_->release(); CE();

    program_->release();

    qDebug() << "Hello from paint plugin";
}

bool Brush3D::mouseClickEvent(QMouseEvent * event){

    screenToRay(event->x(), event->y(), glwidget_->width(), glwidget_->height(),
                      glwidget_->camera_.modelviewMatrix(),
                      glwidget_->camera_.projectionMatrix(),
                      p1,
                      p2);

    int idx = pick(event->x(), event->y(), glwidget_->width(),
                   glwidget_->height(), 1e-04,
                   glwidget_->camera_.projectionMatrix(),
                   glwidget_->camera_.modelviewMatrix(),
                   cl_->active_);

    if(idx == -1)
        return true;

    std::shared_ptr<std::vector<int>> indices;
    indices.reset(new std::vector<int>());

    std::vector<float> distsq;
    cl_->active_->getOctree()->radiusSearch(idx, 0.5, *indices, distsq);

    for(int idx : *indices){
        PointFlags & flag = cl_->active_->flags_[idx];
        flag = PointFlags(uint8_t(flag) | uint8_t(PointFlags::selected));
    }

    cl_->active_->ed_->emitflagUpdate(indices);
    return true;
}

bool Brush3D::mouseMoveEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();
    return true;
}

bool Brush3D::mousePressEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();
    mouse_down_pos_ = last_mouse_pos_;
    return true;
}

bool Brush3D::mouseReleaseEvent(QMouseEvent * event){
    last_mouse_pos_ << event->x(), event->y();
    float dist = (last_mouse_pos_ - mouse_down_pos_).norm();
    if(dist < 2){
        return mouseClickEvent(event);
    }

    return true;
}

void Brush3D::enable() {
    if(is_enabled_){
        disable();
        return;
    }
    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
    tabs->setCurrentWidget(glwidget_);
    enable_->setChecked(true);
    emit enabling();
    connect(glwidget_, SIGNAL(pluginPaint(Eigen::Affine3f, Eigen::Affine3f)),
            this, SLOT(paint(Eigen::Affine3f, Eigen::Affine3f)),
            Qt::DirectConnection);
    glwidget_->installEventFilter(this);
    connect((QObject *)pm_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void Brush3D::disable() {
    enable_->setChecked(false);
    disconnect((QObject *) pm_, SIGNAL(endEdit()), this, SLOT(disable()));
    disconnect(glwidget_, SIGNAL(pluginPaint(Eigen::Affine3f, Eigen::Affine3f)),
            this, SLOT(paint(Eigen::Affine3f, Eigen::Affine3f)));
    glwidget_->removeEventFilter(this);
    is_enabled_ = false;
}

bool Brush3D::eventFilter(QObject *object, QEvent *event){

    switch(event->type()){
    case QEvent::MouseButtonPress:
        return mousePressEvent(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
        return mouseReleaseEvent(static_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
        return mouseMoveEvent(static_cast<QMouseEvent*>(event));
    }


    /*
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
        qDebug() << "Ate key press" << keyEvent->key();
        return true;
    } else {
        return false;
    }
    */
    return false;

}

Q_EXPORT_PLUGIN2(pnp_brush3d, Brush3D)
