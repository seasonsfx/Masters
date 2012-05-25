#ifndef VIEWPANE_H
#define VIEWPANE_H

#include <QGLWidget>
#include <QGLBuffer>
#include <QGLShaderProgram>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "appdata.h"

class ViewPane : public QGLWidget
{
    Q_OBJECT
private:
    QGLFormat glFormat;

    AppData * app_data;

    bool sampling;
    bool filling;
    int normals_drawn;
    int normal_subsample;

    // Buffer data
    GLuint vertexBufferObject;
    GLuint vao;

    GLuint lineBufferObject;
    GLuint vao2;

    GLuint theProgram;

    GLuint modelToCameraMatrixUnif;
    GLuint cameraToClipMatrixUnif;

    glm::mat4 cameraToClipMatrix;//(1.0f);
    glm::mat4 modelview_mat;//(1.0f);
    glm::vec4 offsetVec;//(0.0f,0.0f,0.0f,0.0f);

    float fFrustumScale;

    QGLShaderProgram m_shader;
    QGLBuffer m_vertexBuffer;

    bool prepareShaderProgram( const QString& vertexShaderPath,
                               const QString& fragmentShaderPath );

protected:
    void initializeGL();
    void resizeGL(int x, int h);
    void paintGL();

public:
    explicit ViewPane(QWidget *parent = 0);
    ViewPane( const QGLFormat& format, QWidget* parent= 0);

signals:

public slots:


};

#endif // VIEWPANE_H
