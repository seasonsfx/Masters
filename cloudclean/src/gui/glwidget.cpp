#include "glwidget.h"
#include <QtOpenGL>
#include <QResource>
#include <cmath>
#include <cstdlib>

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent) {
    //qApp->installEventFilter(this);
    setFocusPolicy(Qt::StrongFocus);
    camera_move_unit = 0.4;
    point_render_size = 1.0f;

    // Mouse move events get tracked even if mouse not down
    setMouseTracking(true);
}

GLWidget::~GLWidget()
{
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(500, 500);
}

QSize GLWidget::sizeHint() const

{
    return QSize(500, 500);
}

void GLWidget::initializeGL()
{
    glClearColor(0.8, 0.8, 0.8, 1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);
    
    //
    // Check resources
    //

    //qDebug() << QString(reinterpret_cast<const char*>(QResource(":/points.vert").data()));

    //
    // Load shader
    //
    bool succ = program_.addShaderFromSourceFile(
                QGLShader::Vertex, ":/points.vert");
    CE();
    qWarning() << program_.log();
    if (!succ) qWarning() << "Shader compile log:" << program_.log();
    succ = program_.addShaderFromSourceFile(
                QGLShader::Fragment, ":/points.frag");
    CE();
    if (!succ) qWarning() << "Shader compile log:" << program_.log();
    succ = program_.link();
    CE();
    if (!succ) {
        qWarning() << "Could not link shader program_:" << program_.log();
        qWarning() << "Exiting...";
        abort();
    }

    succ = program_.bind();
    CE();
    if(!succ)
        qDebug() << "Program not bound to context ";

    //
    // Resolve attributes & uniforms
    //
    attr_vertex_ = program_.attributeLocation("vertex"); RC(attr_vertex_);
    attr_intensity_ = program_.attributeLocation("intensity"); RC(attr_color_index_);
    attr_color_index_ = program_.attributeLocation("color_index"); RC(attr_color_index_);
    uni_sampler_ = program_.uniformLocation("sampler"); RC(uni_sampler_);
    uni_projection_ = program_.uniformLocation("projection"); RC(uni_projection_);
    uni_modelview_ = program_.uniformLocation("modelview"); RC( uni_modelview_);

    program_.release();

    //
    // Create bs data for testing...
    //

    vertices_.clear();
    vertices_ << QVector4D(0.0f, 0.0f, 1.0f, 1.0f);
    vertices_ << QVector4D(0.5f, 0.0f, 1.0f, 1.0f);
    vertices_ << QVector4D(0.0f, -0.5f, 1.0f, 1.0f);
    vertices_ << QVector4D(0.5f, -0.5f, 1.0f, 1.0f);

    color_index_.clear();
    color_index_ << 0;
    color_index_ << 1;
    color_index_ << 2;
    color_index_ << 0;

    colours_.clear();
    colours_ << QVector4D(1.0f, 0.0f, 0.0f, 1.0f);
    colours_ << QVector4D(0.0f, 1.0f, 0.0f, 1.0f);
    colours_ << QVector4D(0.0f, 0.0f, 1.0f, 1.0f);


    //
    // Set camera
    //
    camera_.setDepthRange(0.1f, 100.0f);
    camera_.setAspect(width() / static_cast<float>(height()));
    program_.bind();
    glUniformMatrix4fv(uni_projection_, 1, GL_FALSE,
                       camera_.projectionMatrix().data());
    program_.release();
    //
    // Set up VAO
    //
    program_.bind();
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    //
    // Set up buffers
    //

    label_colours_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    index_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    vertex_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();

    label_colours_->create(); CE();
    label_colours_->bind(); CE();
    label_colours_->allocate(colours_.size()*sizeof(QVector4D)); CE();
    label_colours_->write(0, colours_.constData(),
                          colours_.size()*sizeof(QVector4D)); CE();
    label_colours_->release(); CE();

    vertex_buffer_->create(); CE();
    vertex_buffer_->bind(); CE();
    vertex_buffer_->allocate(vertices_.size()*sizeof(QVector4D)); CE();
    vertex_buffer_->write(0, vertices_.constData(),
                          vertices_.size()*sizeof(QVector4D));
    program_.enableAttributeArray(attr_vertex_); CE();
    program_.enableAttributeArray(attr_intensity_); CE();
    program_.setAttributeBuffer(attr_vertex_, GL_FLOAT, 0, 3, sizeof(float)*4); CE();
    program_.setAttributeBuffer(attr_intensity_, GL_FLOAT, sizeof(float)*3, 1, sizeof(float)*4); CE();
    vertex_buffer_->release(); CE();

    index_buffer_->create(); CE();
    index_buffer_->bind(); CE();
    index_buffer_->allocate(color_index_.size()*sizeof(int));  CE();
    index_buffer_->write(0, color_index_.constData(),
                         color_index_.size()*sizeof(int)); CE();
    program_.enableAttributeArray(attr_color_index_);CE();
    glVertexAttribIPointer(attr_color_index_, 1, GL_INT, 0, 0);
    index_buffer_->release(); CE();

    glGenTextures(1, &texture_id_); CE();

    glPointSize(5);

    glBindVertexArray(0);
    program_.release();
}

void GLWidget::paintGL() {

    // Make sure the labels are updates
    // Make sure nothing has changed

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program_.bind();CE();
    glBindVertexArray(vao_);

    glUniformMatrix4fv(uni_modelview_, 1, GL_FALSE,
                       camera_.modelviewMatrix().data());CE();
    glUniformMatrix4fv(uni_projection_, 1, GL_FALSE,
                       camera_.projectionMatrix().data());

    glUniform1i(uni_sampler_, 0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, label_colours_->bufferId()); CE();

    glDrawArrays(GL_POINTS, 0, vertices_.size()); CE();
    glBindTexture(GL_TEXTURE_BUFFER, 0);CE();

    glBindVertexArray(0);
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
    mouse_drag_start = QVector2D(0.0f, 0.0f);

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
            camera_.adjustPosition(camera_move_unit, 0, 0);
            break;
    case Qt::Key_A:
    case Qt::Key_Left:
            camera_.adjustPosition(-camera_move_unit, 0, 0);
            break;
    case Qt::Key_W:
    case Qt::Key_Up:
            camera_.adjustPosition(0, 0, camera_move_unit);
            break;
    case Qt::Key_S:
    case Qt::Key_Down:
            camera_.adjustPosition(0, 0, -camera_move_unit);
            break;
    case Qt::Key_Q:
            camera_.adjustPosition(0, +camera_move_unit, 0);
            break;
    case Qt::Key_E:
            camera_.adjustPosition(0, -camera_move_unit, 0);
            break;
    case Qt::Key_R:
            if (event->modifiers() == Qt::ControlModifier)
                camera_.setPosition(0, 0, 0);
            break;
    case Qt::Key_Plus:
            if (point_render_size < 30)
                point_render_size++;
            break;
    case Qt::Key_Minus:
            if ( point_render_size > 1 )
                point_render_size--;
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
