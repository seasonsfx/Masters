#include "plugins/brush3d/brush3d.h"
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
#include <QHBoxLayout>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QSpacerItem>
#include <QStackedWidget>
#include <QSlider>
#include <QButtonGroup>
#include <QDockWidget>
#include <QPushButton>
#include <QGridLayout>
#include <QCheckBox>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "utilities/pointpicker.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

QString Brush3D::getName(){
    return "Brush Tool";
}

void Brush3D::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;
    initialized_gl = false;

    enable_ = new QAction(QIcon(":/images/brush.png"), "Brush Tool", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);

    last_picked_point_ = -1;
    last_rad_ = 0;
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

    QSlider * slider = new QSlider(settings_);
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(1, 300);
    slider->setSingleStep(1);
    slider->setToolTip("Radius in cm");
    slider->setValue(radius_*100);
    slider->setTickPosition(QSlider::TicksBelow);

    layout->addWidget(new QLabel("Radius:", settings_));
    layout->addWidget(slider);

    layout->addWidget(new QLabel("Depth correct:", settings_));
    QCheckBox * cb = new QCheckBox(settings_);
    depth_adjust_ = true;
    cb->setChecked(depth_adjust_);
    layout->addWidget(cb);
    cb->connect(cb, &QCheckBox::toggled, [this] (bool on) {
        depth_adjust_ = on;
    });

    layout->addItem(new QSpacerItem(0, 0, QSizePolicy::Maximum, QSizePolicy::Maximum));

    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(setRad(int)));
}

void Brush3D::setRad(int val) {
    radius_ = val/100.0;
}

void Brush3D::cleanup(){
    disconnect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
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

void Brush3D::paint(const Eigen::Affine3f& proj, const Eigen::Affine3f& mv){
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
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0); CE();

    glLineWidth(2);
    glDrawArrays(GL_LINES, 0, 2); CE();

    line_->release(); CE();

    program_->release();

    qDebug() << "Hello from paint plugin";
}

void Brush3D::select2D(int x, int y){
    boost::shared_ptr<PointCloud> pc = cl_->active_;

    Eigen::Vector3f coord;
    coord << 2.0f* (x/float(flatview_->width()) - 0.5),
        -2.0f* (y/float(flatview_->height()) - 0.5), 1;
    coord = flatview_->getCamera().inverse() * coord;

    coord[1] = cl_->active_->scan_height()-coord[1];

    boost::shared_ptr<std::vector<int> > indices;
    indices.reset(new std::vector<int>());

    int rad = radius_ * 1000;

    for(int x = -rad/2; x < rad/2; x++){
        for(int y = -rad/2; y < rad/2; y++){
            if(x*x + y*y > (rad/2.0f)*(rad/2.0f) )
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

    bool negative_select = QApplication::keyboardModifiers() == Qt::ControlModifier;
    core_->us_->push(new Select(pc, indices, core_->mw_->deselect_ || negative_select, core_->mw_->select_mask_));

}

int Brush3D::select3D(float x, float y){
    int idx = pick(x, y, glwidget_->width(),
                   glwidget_->height(), 1e-04,
                   glwidget_->camera_.projectionMatrix(),
                   glwidget_->camera_.modelviewMatrix(),
                   cl_->active_);

    if(idx == -1)
        return -1;

    Eigen::Vector3f p1, p2;
    int r = radius_ * 100;

    float rad;

    if(depth_adjust_){
        GLfloat wz;
        glReadPixels(x, glwidget_->height() - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &wz);
        // picked radius
        screenToWorld(x-r, y-r, x+r, y+r, glwidget_->width(), glwidget_->height(), Eigen::Affine3f::Identity(), glwidget_->camera_.projectionMatrix(), p1, p2, wz);
        rad = (p2-p1).norm()/2.0f;
    } else {
        // near radius
        screenToWorld(x-r, y-r, x+r, y+r, glwidget_->width(), glwidget_->height(), Eigen::Affine3f::Identity(), glwidget_->camera_.projectionMatrix(), p1, p2, 0);
        rad = (p2-p1).norm()/2.0f;
        rad = rad * 10;
    }

    last_rad_ = rad;

    boost::shared_ptr<std::vector<int> > indices;
    indices.reset(new std::vector<int>());

    std::vector<float> distsq;
    cl_->active_->octree()->radiusSearch(idx, rad, *indices, distsq);

    bool negative_select = QApplication::keyboardModifiers() == Qt::ControlModifier;

    core_->us_->push(new Select(cl_->active_, indices, core_->mw_->deselect_ || negative_select, core_->mw_->select_mask_));

    return idx;
}

bool Brush3D::mouseClickEvent(QMouseEvent * event){
    return true;
}

bool Brush3D::mouseMoveEvent(QMouseEvent * event) {
    // How is the mouse moving?
    Eigen::Vector2d pos(event->x(), event->y());
    Eigen::Vector2d last_pos = last_mouse_pos_;

    last_mouse_pos_ << event->x(), event->y();
    if(event->buttons() != Qt::LeftButton && !event->modifiers())
        return false;
    if(event->buttons() != Qt::LeftButton && event->modifiers() == Qt::Key_Control)
        return true;
    if(cl_->clouds_.size() == 0)
        return false;

    if(is3d()){
        if(event->buttons()){
            if(last_picked_point_ != -1){

                Eigen::Vector3f p1 = cl_->active_->at(last_picked_point_).getArray3fMap();
                last_picked_point_ = select3D(event->x(), event->y());
                Eigen::Vector3f p2 = cl_->active_->at(last_picked_point_).getArray3fMap();

                Eigen::Vector3f diff = p2-p1;
                float len = diff.norm();
                qDebug() << "len" << len << "< " << (radius_*100)/4;
                if(len > last_rad_/4) {
                    float dist = 0;
                    Eigen::Vector3f dir = diff.normalized();
                    // Interpolate
                    while(dist < len){
                        qDebug() << "dist" << dist;
                        Eigen::Vector3f p = p1 + dist*dir;
                        dist+=last_rad_/4;

                        pcl::PointXYZI q;
                        q.getArray3fMap() = p;

                        // calculate radius

                        int viewport[4] = {0, 0, glwidget_->x(), glwidget_->y()};
                        double wx, wy, wz;

                        double mv [16];
                        double proj [16];

                        Eigen::Affine3f t = (glwidget_->camera_.modelviewMatrix() * cl_->active_->modelview());
                        Eigen::Affine3f t2 = glwidget_->camera_.projectionMatrix();

                        const float * mvt = t.data();
                        const float * projt = t2.data();

                        for(int i = 0; i < 16; i++){
                            mv[i] = mvt[i];
                            proj[i] = projt[i];
                        }

                        gluProject(p.x(), p.y(), p.z(),
                            mv,
                            proj,
                            viewport,
                            &wx, &wy, &wz);




//                        boost::shared_ptr<std::vector<int> > indices;
//                        indices.reset(new std::vector<int>());

//                        std::vector<float> distsq;
//                        cl_->active_->octree()->radiusSearch(q, last_rad_, *indices, distsq);

//                        bool negative_select = QApplication::keyboardModifiers() == Qt::ControlModifier;
//                        core_->us_->push(new Select(cl_->active_, indices, core_->mw_->deselect_ || negative_select, core_->mw_->select_mask_));



                        last_picked_point_ = select3D(wx, wy);
                    }
                }

            } else {
                last_picked_point_ = select3D(event->x(), event->y());
            }
        }
    } else {

        if(event->buttons()){
            Eigen::Vector2d diff(pos - last_pos);
            float len = diff.norm();
            if(len > (radius_*1000)/4) {
                float dist = 0;
                Eigen::Vector2d dir = diff.normalized();
                // Interpolate
                while(dist <= len){
                    Eigen::Vector2d p = last_pos + dist*dir;
                    dist+=5;
                    select2D(p.x(), p.y());
                }
            }
            select2D(event->x(), event->y());
        }
    }

    return true;
}

bool Brush3D::mousePressEvent(QMouseEvent * event) {
    if(event->buttons() != Qt::LeftButton)
        return false;
    if(cl_->clouds_.size() == 0)
        return false;
    core_->us_->beginMacro("3d Select");
    if(is3d()){
        last_picked_point_ = select3D(event->x(), event->y());
    } else {
        select2D(event->x(), event->y());
    }
    last_mouse_pos_ << event->x(), event->y();
    mouse_down_pos_ = last_mouse_pos_;
    return true;
}

bool Brush3D::mouseReleaseEvent(QMouseEvent * event){
    core_->us_->endMacro();
    last_mouse_pos_ << event->x(), event->y();
    float dist = (last_mouse_pos_ - mouse_down_pos_).norm();
    last_picked_point_ = -1;
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
//    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
//    tabs->setCurrentWidget(glwidget_);
//    enable_->setChecked(true);

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);

    emit enabling();
    //connect(glwidget_, SIGNAL(pluginPaint(Eigen::Affine3f, Eigen::Affine3f)),
    //        this, SLOT(paint(Eigen::Affine3f, Eigen::Affine3f)),
    //        Qt::DirectConnection);
    glwidget_->installEventFilter(this);
    flatview_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void Brush3D::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    //disconnect(glwidget_, SIGNAL(pluginPaint(Eigen::Affine3f, Eigen::Affine3f)),
    //        this, SLOT(paint(Eigen::Affine3f, Eigen::Affine3f)));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

void Brush3D::setSelectMask(uint8_t mask){
    core_->mw_->setSelectMask(mask);
}

bool Brush3D::is3d(){
    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
    return tabs->currentIndex() == tabs->indexOf(glwidget_);
}

bool Brush3D::eventFilter(QObject *object, QEvent *event){

    // Bypass plugin via shift
    if(QApplication::keyboardModifiers() == Qt::SHIFT || !core_->mw_->edit_mode_)
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
            setSelectMask(1);
            return true;
        case Qt::Key_2:
            setSelectMask(2);
            return true;
        case Qt::Key_3:
            setSelectMask(4);
            return true;
        case Qt::Key_4:
            setSelectMask(8);
            return true;
        case Qt::Key_5:
            setSelectMask(16);
            return true;
        case Qt::Key_6:
            setSelectMask(32);
            return true;
        case Qt::Key_7:
            setSelectMask(64);
            return true;
        case Qt::Key_8:
            setSelectMask(128);
            return true;
        }

    default:
        return false;
    }
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
