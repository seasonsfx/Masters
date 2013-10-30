#include "glwidget.h"
#include <QResource>
#include <QKeyEvent>
#include <QCoreApplication>
#include <QWindow>
#include <cmath>
#include <cstdlib>
#include "utilities/pointpicker.h"

using namespace Eigen;

GLWidget::GLWidget(QGLFormat &fmt, CloudList *cl,
                   LayerList *ll, QWidget *parent)
    : QGLWidget(fmt, parent) {
    setFocusPolicy(Qt::StrongFocus);
    translate_unit_ = 0.4;
    point_render_size_ = 4.0f;

    cl_ = cl;
    ll_ = ll;
    setMouseTracking(true);
    setContextMenuPolicy(Qt::CustomContextMenu);
    setAutoFillBackground(false);
    setAutoBufferSwap(false);
    gl_init_ = false;
}

GLWidget::~GLWidget() {
}

void GLWidget::setGLD(GLData * gld){
    gld_ = gld;
}

QSize GLWidget::minimumSizeHint() const {
    return QSize(500, 500);
}

QSize GLWidget::sizeHint() const {
    return QSize(700, 500);
}

void GLWidget::initializeGL() {
    #if defined(Q_OS_WIN32)
        glewExperimental = true;
        GLenum GlewInitResult = glewInit();
        if (GlewInitResult != GLEW_OK) {
            const GLubyte* errorStr = glewGetErrorString(GlewInitResult);
            size_t size = strlen(reinterpret_cast<const char*>(errorStr));
            qDebug() << "Glew error "
                     << QString::fromUtf8(
                            reinterpret_cast<const char*>(errorStr), size);
        }
    #endif

    CE();

    glClearColor(0.8, 0.8, 0.8, 1.0);
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_POINT_SMOOTH);
    
    //
    // Load shader program
    //
    bool succ = program_.addShaderFromSourceFile(
                QGLShader::Vertex, ":/points.vert"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_.log();
    succ = program_.addShaderFromSourceFile(
                QGLShader::Fragment, ":/points.frag"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_.log();
    succ = program_.link(); CE();
    if (!succ) {
        qWarning() << "Could not link shader program_:" << program_.log();
        qWarning() << "Exiting...";
        abort();
    }

    //emit pluginPaint(camera_.projectionMatrix(), camera_.modelviewMatrix());

    //
    // Resolve uniforms
    //
    program_.bind(); CE();
    uni_sampler_ = program_.uniformLocation("sampler"); RC(uni_sampler_);
    uni_projection_ = program_.uniformLocation("projection"); RC(uni_projection_);
    uni_modelview_ = program_.uniformLocation("modelview"); RC(uni_modelview_);

    program_.release(); CE();

    //
    // Set camera
    //
    camera_.setDepthRange(0.1f, 100.0f);
    camera_.setAspect(width() / static_cast<float>(height()));

    //
    // Set up textures & point size
    //
    glGenTextures(1, &texture_id_); CE();
    glPointSize(point_render_size_);

    // Generate vao
    glGenVertexArrays(1, &vao_);
    gl_init_ = true;
}


void GLWidget::paintEvent(QPaintEvent *event) {
    if(!isValid())
        return;

    makeCurrent();
    if(!gl_init_)
        initializeGL();

    //makeCurrent();
    // Make sure the labels are updates
    // Make sure nothing has changed

    QPainter p(this);

    QRadialGradient gradient;
    gradient.setCoordinateMode(QGradient::ObjectBoundingMode);
    gradient.setCenter(0.45, 0.50);
    gradient.setFocalPoint(0.40, 0.45);
    gradient.setColorAt(0.0, QColor(105, 146, 182));
    gradient.setColorAt(0.4, QColor(81, 113, 150));
    gradient.setColorAt(0.8, QColor(16, 56, 121));

    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setPen(Qt::NoPen);
    p.setBrush(gradient);
    p.drawRect(0, 0, size().width(), size().height());

    p.beginNativePainting();

    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    program_.bind(); CE();

    glUniformMatrix4fv(uni_modelview_, 1, GL_FALSE,
                       camera_.modelviewMatrix().data());CE();
    glUniformMatrix4fv(uni_projection_, 1, GL_FALSE,
                       camera_.projectionMatrix().data());

    glUniform1i(uni_sampler_, 0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, gld_->color_lookup_buffer_->bufferId()); CE();

    // Draw all clouds
    for(std::pair<boost::shared_ptr<PointCloud>, boost::shared_ptr<CloudGLData> > pair: gld_->cloudgldata_) {
        boost::shared_ptr<PointCloud> pc = pair.first;
        boost::shared_ptr<CloudGLData> cd = pair.second;

        if(!pc->isVisible())
            continue;

        glBindVertexArray(vao_);

        // Point buffer
        cd->point_buffer_->bind(); CE();
        glEnableVertexAttribArray(0); CE();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*4, 0); CE();
        glEnableVertexAttribArray(1); CE();
        int offset = sizeof(float)*3;
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float)*4,
                              reinterpret_cast<const void *>(offset)); CE();
        cd->point_buffer_->release(); CE();

        // Label buffer
        cd->label_buffer_->bind(); CE();
        glEnableVertexAttribArray(2); CE(); CE();
        glVertexAttribIPointer(2, 1, GL_SHORT, 0, 0); CE();
        cd->label_buffer_->release(); CE();

        // Flag buffer
        cd->flag_buffer_->bind(); CE();
        glEnableVertexAttribArray(3); CE();
        glVertexAttribIPointer(3, 1, GL_BYTE, 0, 0); CE();
        cd->flag_buffer_->release(); CE();

        glUniformMatrix4fv(uni_modelview_, 1, GL_FALSE,
                           (camera_.modelviewMatrix()*pc->modelview())
                           .data());CE();

        cd->draw(vao_);
        glBindVertexArray(0);
    }

    glBindTexture(GL_TEXTURE_BUFFER, 0); CE();

    p.endNativePainting();
    p.end();

    emit pluginPaint(camera_.projectionMatrix(), camera_.modelviewMatrix());
    glFinish();
    swapBuffers();
}

void GLWidget::resizeGL(int width, int height) {
    camera_.setAspect(width / static_cast<float>(height));
    glViewport(0, 0, width, qMax(height, 1));
}

void GLWidget::mouseDoubleClickEvent(QMouseEvent * event) {

}

void GLWidget::mouseMoveEvent(QMouseEvent * event) {
    float damp = 0.005;
    Vector2f rot(event->x()-last_mouse_pos_.x(), event->y()-last_mouse_pos_.y());
    last_mouse_pos_ << event->x(), event->y();
    rot*=damp;

    if(event->buttons() == Qt::LeftButton && event->modifiers() != Qt::ControlModifier){
        camera_.rotate2D(rot.x(), rot.y());
    }
    else if(event->buttons() /* ==  Qt::RightButton */ ||   event->modifiers() == Qt::ControlModifier){
        boost::shared_ptr<PointCloud> pc = cl_->active_;
        pc->rotate2D(rot.x(), -rot.y());
        //qDebug() << "Why?";
    }

    if(event->buttons())
        update();
}

void GLWidget::mousePressEvent(QMouseEvent * event) {
    mouse_drag_start_ = QVector2D(0.0f, 0.0f);
    last_mouse_pos_ << event->x(), event->y();
}

void GLWidget::mouseReleaseEvent(QMouseEvent * event) {
    //float dist = (Eigen::Vector2d(event->x(), event->y()) - last_mouse_pos_).norm();
    last_mouse_pos_ << event->x(), event->y();

    update();
}

void GLWidget::wheelEvent(QWheelEvent * event) {
    //float x = 2.0f* ((event->x()/float(width())) - 0.5);
    //float y = -2.0f* ((event->y()/float(height())) - 0.5);

    camera_.adjustFov(event->delta());
    update();
}

void GLWidget::keyPressEvent(QKeyEvent * event) {
    switch (event->key()) {

    //
    // Translations:
    //
    case Qt::Key_D:
    case Qt::Key_Right:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(translate_unit_ * camera_.rotation_.matrix().inverse() * Vector3f::UnitX());
        else if(event->modifiers() != Qt::ControlModifier)
            camera_.translate(-translate_unit_ * Vector3f::UnitX());
        break;
    case Qt::Key_A:
    case Qt::Key_Left:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(-translate_unit_ * camera_.rotation_.matrix().inverse() * Vector3f::UnitX());
        else if (event->modifiers() != Qt::ControlModifier)
            camera_.translate(translate_unit_ * Vector3f::UnitX());
        break;
    case Qt::Key_W:
    case Qt::Key_Up:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(-translate_unit_ * camera_.rotation_.matrix().inverse() * Vector3f::UnitZ());
        else if (event->modifiers() != Qt::ControlModifier)
            camera_.translate(translate_unit_ * Vector3f::UnitZ());
        break;
    case Qt::Key_S:
    case Qt::Key_Down:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(translate_unit_ * camera_.rotation_.matrix().inverse() * Vector3f::UnitZ());
        else if (event->modifiers() != Qt::ControlModifier)
            camera_.translate(-translate_unit_ * Vector3f::UnitZ());
        break;
    case Qt::Key_Q:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(-translate_unit_ * camera_.rotation_.matrix().inverse() * Vector3f::UnitY());
        else if (event->modifiers() != Qt::ControlModifier)
            camera_.translate(translate_unit_ * Vector3f::UnitY());
        break;
    case Qt::Key_E:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(translate_unit_ * camera_.rotation_.matrix().inverse() * Vector3f::UnitY());
        else if (event->modifiers() != Qt::ControlModifier)
            camera_.translate(-translate_unit_ * Vector3f::UnitY());
        break;

    //
    // Reset
    //
    case Qt::Key_R:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            camera_.setPosition(0, 0, 0);
        break;

    //
    // Zoom
    //
    case Qt::Key_Equal:
        //if(event->modifiers() != Qt::ShiftModifier)
        //    break;
    case Qt::Key_Plus:
        if (point_render_size_ < 30)
            point_render_size_++;
        glPointSize(point_render_size_);
        break;
    case Qt::Key_Minus:
        if ( point_render_size_ > 1 )
            point_render_size_--;
        glPointSize(point_render_size_);
        break;
    }
    update();
}


bool GLWidget::eventFilter(QObject *object, QEvent *event) {
    Q_UNUSED(object);
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        keyPressEvent(keyEvent);
        return true;
    }
    return false;
}

void GLWidget::contextMenu(const QPoint &pos) {

}

GLData * GLWidget::getGLData(){
    return gld_;
}
