#include <QGLFormat>
#include <cmath>
#include "viewpane.h"

ViewPane::ViewPane( const QGLFormat& format, QWidget* parent):
    QGLWidget( format, parent ),
    m_vertexBuffer( QGLBuffer::VertexBuffer )
{
    std::cout<< "BALLS" << std::endl;
}

ViewPane::ViewPane(QWidget *parent): m_vertexBuffer( QGLBuffer::VertexBuffer )
{
    app_data = AppData::Instance();

    sampling = false;
    filling = false;
    normals_drawn = 0;
    normal_subsample = 20; // Used for drawing normals
    fFrustumScale = 1.0f;

    cameraToClipMatrix = glm::mat4(1.0f);
    modelview_mat = glm::mat4(1.0f);
    offsetVec = glm::vec4(0.0f,0.0f,0.0f,0.0f);


    float fzNear = 0.000001f; float fzFar = 100.0f;

    cameraToClipMatrix[0].x = fFrustumScale;
    cameraToClipMatrix[1].y = fFrustumScale;
    cameraToClipMatrix[2].z = (fzFar + fzNear) / (fzNear - fzFar);
    cameraToClipMatrix[2].w = -1.0f;
    cameraToClipMatrix[3].z = (2 * fzFar * fzNear) / (fzNear - fzFar);

    glFormat.setVersion( 3, 3 );
    //glFormat.setProfile( QGLFormat::CoreProfile ); // Requires >=Qt-4.8.0
    glFormat.setSampleBuffers( true );
    glFormat.setDoubleBuffer( true );
    glFormat.setDepth( true );
    glFormat.setStencil( true );
    glFormat.setAlpha( true );
    QGLWidget(glFormat, parent);
}

void ViewPane::initializeGL(){
    QGLFormat glFormat = QGLWidget::format();
        if ( !glFormat.sampleBuffers() )
            qWarning() << "Could not enable sample buffers";

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Prepare a complete shader program...
    if ( !prepareShaderProgram( "shaders/shader.vert", "shaders/shader.frag" ) )
        return;

    m_vertexBuffer.create();
    m_vertexBuffer.setUsagePattern( QGLBuffer::DynamicDraw );

    /*float points[] = { -0.5f, -0.5f, 0.0f, 1.0f,
                            0.5f, -0.5f, 0.0f, 1.0f,
                            0.0f,  0.5f, 0.0f, 1.0f };
    m_vertexBuffer.allocate( points, 3 * 4 * sizeof( float ) );
*/
    if ( !m_vertexBuffer.bind() )
    {
        qWarning() << "Could not bind vertex buffer to the context";
        return;
    }

    /*m_vertexBuffer.allocate(app_data->cloud->points.size() * sizeof(float) * 4);
    float data[4];

    for (int i = 0; i < app_data->cloud->size(); i++)
    {
        data[0] = app_data->cloud->points[i].x;
        data[1] = app_data->cloud->points[i].y;
        data[2] = app_data->cloud->points[i].z;
        data[3] = app_data->cloud->points[i].intensity;

        int offset = 4*sizeof(float)*i;
        m_vertexBuffer.write(offset, reinterpret_cast<const void *> ( data), sizeof(data));
    }
*/


    // Bind the shader program so that we can associate variables from
    // our application to the shaders
    if ( !m_shader.bind() )
    {
        qWarning() << "Could not bind shader program to context";
        return;
    }

    // Enable the "vertex" attribute to bind it to our currently bound
    // vertex buffer.
    m_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
    m_shader.enableAttributeArray( "vertex" );

    // Set uniforms
    //m_shader.setUniformValueArray ( "cameraToClipMatrix", glm::value_ptr(cameraToClipMatrix), 4, 4 );
    //m_shader.setUniformValueArray ( "modelToCameraMatrix", glm::value_ptr(modelview_mat), 4, 4 );
}

void ViewPane::resizeGL(int w, int h){
    cameraToClipMatrix[0].x = fFrustumScale * (h / (float)w);
    cameraToClipMatrix[1].y = fFrustumScale;

    //m_shader.setUniformValueArray ( "cameraToClipMatrix", glm::value_ptr(cameraToClipMatrix), 4, 4 );

    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
}
void ViewPane::paintGL(){
    // Clear the buffer with the current clearing color
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Draw stuff
    //glDrawArrays( GL_POINTS, 0, app_data->cloud->points.size() );
    glDrawArrays( GL_POINTS, 0, 3 );

    std::cout<< "PAINT" << std::endl;
}

bool ViewPane::prepareShaderProgram( const QString& vertexShaderPath,
                                     const QString& fragmentShaderPath )
{
    // First we load and compile the vertex shader...
    bool result = m_shader.addShaderFromSourceFile( QGLShader::Vertex, vertexShaderPath );
    if ( !result )
        qWarning() << m_shader.log();

    // ...now the fragment shader...
    result = m_shader.addShaderFromSourceFile( QGLShader::Fragment, fragmentShaderPath );
    if ( !result )
        qWarning() << m_shader.log();

    // ...and finally we link them to resolve any references.
    result = m_shader.link();
    if ( !result )
        qWarning() << "Could not link shader program:" << m_shader.log();

    return result;
}
