#include "glwidget.h"
#include <QtOpenGL>
#include <QResource>
#include <cmath>
#include <cstdlib>

GLWidget::GLWidget(QGLFormat &fmt, std::shared_ptr<CloudList> &cl,
                   std::shared_ptr<LayerList> &ll, QWidget *parent)
    : QGLWidget(fmt, parent) {
    setFocusPolicy(Qt::StrongFocus);
    camera_move_unit_ = 0.4;
    point_render_size_ = 4.0f;

    cl_ = cl;
    ll_ = ll;
    setMouseTracking(true);
}

GLWidget::~GLWidget() {
}

void GLWidget::setGLD(std::shared_ptr<GLData> gld){
    gld_ = gld;
}

QSize GLWidget::minimumSizeHint() const {
    return QSize(500, 500);
}

QSize GLWidget::sizeHint() const {
    return QSize(700, 500);
}

void GLWidget::initializeGL() {
    glClearColor(0.8, 0.8, 0.8, 1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);
    
    //
    // Load shader program
    //
    bool succ = program_.addShaderFromSourceFile(
                QGLShader::Vertex, ":/points.vert"); CE();
    qWarning() << program_.log();
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
    //
    // Resolve uniforms
    //
    program_.bind(); CE();
    uni_sampler_ = program_.uniformLocation("sampler"); RC(uni_sampler_);
    uni_projection_ = program_.uniformLocation("projection"); RC(uni_projection_);
    uni_modelview_ = program_.uniformLocation("modelview"); RC(uni_modelview_);
    uni_select_color_ = program_.uniformLocation("select_color"); RC(uni_select_color_);
    program_.release();
    //
    // Set camera
    //
    camera_.setDepthRange(0.1f, 100.0f);
    camera_.setAspect(width() / static_cast<float>(height()));
    //
    // Selection color
    //
    program_.bind(); CE();
    glUniform4fv(uni_select_color_, 1, gld_->selection_color_); CE();
    program_.release(); CE();
    //
    // Set up textures & point size
    //
    glGenTextures(1, &texture_id_); CE();
    glPointSize(point_render_size_);

    // Generate vao
    glGenVertexArrays(1, &vao_);
}


void GLWidget::paintGL() {

    // Make sure the labels are updates
    // Make sure nothing has changed

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program_.bind(); CE();

    glUniformMatrix4fv(uni_modelview_, 1, GL_FALSE,
                       camera_.modelviewMatrix().data());CE();
    glUniformMatrix4fv(uni_projection_, 1, GL_FALSE,
                       camera_.projectionMatrix().data());

    glUniform1i(uni_sampler_, 0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, gld_->color_lookup_buffer_->bufferId()); CE();

    // TODO(Rickert): Cloud position in world space

    // Draw all clouds
    for(std::pair<std::shared_ptr<PointCloud>, std::shared_ptr<CloudGLData> > pair: gld_->cloudgldata_) {
        std::shared_ptr<CloudGLData> cd = pair.second;

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

        cd->draw(vao_);
        glBindVertexArray(0);
    }

    glBindTexture(GL_TEXTURE_BUFFER, 0); CE();

    program_.release();
}

void GLWidget::resizeGL(int width, int height) {
    camera_.setAspect(width / static_cast<float>(height));
    glViewport(0, 0, width, qMax(height, 1));
}

void GLWidget::mouseDoubleClickEvent(QMouseEvent * event) {

}

void GLWidget::mouseMoveEvent(QMouseEvent * event) {
    if(event->buttons() & Qt::RightButton)
        camera_.mouseMove(event->x(), event->y());
    if(event->buttons() &  Qt::LeftButton)
        camera_.mouseMove(event->x(), event->y());

    if(event->buttons())
        updateGL();
}

void GLWidget::mousePressEvent(QMouseEvent * event) {
    mouse_drag_start_ = QVector2D(0.0f, 0.0f);
    camera_.mouseDown(event->x(), event->y(), event->button());
}

void GLWidget::mouseReleaseEvent(QMouseEvent * event) {
    if (event->button() == Qt::RightButton)
        camera_.mouseRelease(event->x(), event->y());
    else if (event->button() == Qt::LeftButton)
        camera_.mouseRelease(event->x(), event->y());
    updateGL();
}

void GLWidget::wheelEvent(QWheelEvent * event) {
    camera_.mouseWheel(event->delta());
    updateGL();
}

void GLWidget::keyPressEvent(QKeyEvent * event) {
    switch (event->key()) {
    case Qt::Key_Escape:
            QCoreApplication::instance()->quit();
            break;
    case Qt::Key_D:
    case Qt::Key_Right:
            camera_.adjustPosition(camera_move_unit_, 0, 0);
            break;
    case Qt::Key_A:
    case Qt::Key_Left:
            camera_.adjustPosition(-camera_move_unit_, 0, 0);
            break;
    case Qt::Key_W:
    case Qt::Key_Up:
            camera_.adjustPosition(0, 0, camera_move_unit_);
            break;
    case Qt::Key_S:
    case Qt::Key_Down:
            camera_.adjustPosition(0, 0, -camera_move_unit_);
            break;
    case Qt::Key_Q:
            camera_.adjustPosition(0, +camera_move_unit_, 0);
            break;
    case Qt::Key_E:
            camera_.adjustPosition(0, -camera_move_unit_, 0);
            break;
    case Qt::Key_R:
            if (event->modifiers() == Qt::ControlModifier)
                camera_.setPosition(0, 0, 0);
            break;
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
    updateGL();
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
