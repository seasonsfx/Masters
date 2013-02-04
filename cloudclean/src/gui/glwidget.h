#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "glheaders.h"
#include <memory>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include "gui/camera.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

	void resetRotationMatrix();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

private:
    QVector<QVector4D> vertices_;
    QVector<QVector4D> colours_;
    QVector<int> color_index_;

    Camera camera_;

    QGLShaderProgram program_;
    std::shared_ptr<QGLBuffer> label_colours_;
    std::shared_ptr<QGLBuffer> index_buffer_;  // Used for masking?
    std::shared_ptr<QGLBuffer> vertex_buffer_;

    int attr_vertex_;
    int attr_color_index_;
    int uni_sampler_;
    int uni_projection_;
    int uni_modelview_;

    GLuint texture_id_;
    GLuint vao_;

    // So basically here we need qt datastructures that repreent the model state
    // On every draw the data should the model should be checked for modifications
    // Should modifications happen these structure need to be updated

};

#endif

