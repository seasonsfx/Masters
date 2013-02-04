#include "glwidget.h"
#include <QtOpenGL>
#include <QResource>
#include <cmath>
#include <cstdlib>

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent) {
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
    if (!succ) qWarning() << program_.log();
    succ = program_.addShaderFromSourceFile(
                QGLShader::Fragment, ":/points.frag");
    CE();
    if (!succ) qWarning() << program_.log();
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
    //attr_vertex_ = glGetAttribLocation(program_.programId(), "vertex"); CE(); RC(attr_vertex_);
    attr_vertex_ = program_.attributeLocation("vertex"); RC(attr_vertex_);
    attr_color_index_ = program_.attributeLocation("color_index"); RC(attr_color_index_);
    uni_sampler_ = program_.uniformLocation("sampler"); RC(uni_sampler_);
    uni_projection_ = program_.uniformLocation("projection"); RC(uni_projection_);
    uni_modelview_ = program_.uniformLocation("modelview"); RC( uni_modelview_);


    //
    // Create bs data for testing...
    //

    vertices_.clear();
    vertices_ << QVector4D(0.25f, 0.75f, 1.0f, 0.5f);
    vertices_ << QVector4D(-0.25f, 0.75f, 1.0f, 0.5f);
    vertices_ << QVector4D(0.25f, 0.25f, 1.0f, 0.5f);
    vertices_ << QVector4D(-0.25f, 0.25f, 1.0f, 0.5f);

    color_index_.clear();
    color_index_ << 0;
    color_index_ << 1;
    color_index_ << 2;
    color_index_ << 2;

    colours_.clear();
    colours_ << QVector4D(1.0f, 0.75f, 1.0f, 1.0f);
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
    // Set up buffers
    //

    label_colours_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    index_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    vertex_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();

    label_colours_->create(); CE();
    vertex_buffer_->create(); CE();
    index_buffer_->create(); CE();

    label_colours_->bind(); CE();
    label_colours_->allocate(colours_.size()*sizeof(QVector4D)); CE();
    label_colours_->write(0, colours_.constData(),
                          colours_.size()*sizeof(QVector4D)); CE();
    label_colours_->release(); CE();

    vertex_buffer_->bind(); CE();
    vertex_buffer_->allocate(vertices_.size()*sizeof(QVector4D)); CE();
    vertex_buffer_->write(0, vertices_.constData(),
                          vertices_.size()*sizeof(QVector4D));
    vertex_buffer_->release(); CE();


    index_buffer_->bind(); CE();
    index_buffer_->allocate(color_index_.size()*sizeof(int));  CE();
    index_buffer_->write(0, color_index_.constData(),
                         color_index_.size()*sizeof(int)); CE();

    index_buffer_->release(); CE();

    glGenTextures(1, &texture_id_); CE();

    //
    // Set up VAO
    //

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    vertex_buffer_->bind(); CE();
    program_.enableAttributeArray(attr_vertex_); CE();
    program_.setAttributeBuffer(attr_vertex_, GL_FLOAT, 0, 4); CE();
    vertex_buffer_->release(); CE();

    index_buffer_->bind();CE();
    program_.enableAttributeArray(attr_color_index_);CE();
    program_.setAttributeBuffer(attr_color_index_, GL_INT, 0, 1);CE();
    index_buffer_->release();CE();

    glBindVertexArray(0);
}

void GLWidget::paintGL() {

    // Make sure the labels are updates
    // Make sure nothing has changed

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program_.bind();CE();
    glBindVertexArray(vao_);

    glUniformMatrix4fv(uni_modelview_, 1, GL_FALSE,
                       camera_.modelviewMatrix().data());CE();

    glUniform1i(uni_sampler_, 0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, label_colours_->bufferId()); CE();


    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDrawArrays(GL_POINTS, 0, vertices_.size()); CE();
    glBindTexture(GL_TEXTURE_BUFFER, 0);CE();

    glBindVertexArray(0);
    program_.release();

}

void GLWidget::resizeGL(int width, int height) {
    camera_.setAspect(width / static_cast<float>(height));
    program_.bind();
    glUniformMatrix4fv(uni_projection_, 1, GL_FALSE,
                       camera_.projectionMatrix().data());
    program_.release();
    //int side = qMin(width, height);
    //glViewport((width - side) / 2, (height - side) / 2, side, side);
    glViewport(0, 0, width, qMax(height, 1));
}
