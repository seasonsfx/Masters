#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include <GL/glu.h>

#include <QIcon>
#include <QDebug>
#include <QResource>
#include <QAbstractItemView>
#include <QTime>

#include "viz_normals.h"
#include "utilities.h"
#include "layer.h"
#include "glarea.h"
#include "cloudmodel.h"

VizNormals::VizNormals()
{

    editSample = new QAction(QIcon(":/images/normal.png"), "Show normals", this);
    actionList << editSample;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

VizNormals::~VizNormals()
{
}

bool VizNormals::StartViz(QAction *action, CloudModel *cm, GLArea *glarea){

    if(!normal_buffer.isCreated()){

        // Setup normal shader
        assert(glarea->prepareShaderProgram(normal_shader, ":/shaders/normals.vert", ":/shaders/normals.frag", ":/shaders/normals.geom" ) );

        if ( !normal_shader.bind() )
        {
            qWarning() << "Could not bind shader program to context";
            assert(false);
        }
        normal_shader.enableAttributeArray( "vertex" );
        normal_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 3 );
        glUniformMatrix4fv(normal_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, glarea->camera.projectionMatrix().data());
        glError("134");
        normal_shader.release();

        // Create buffer
        normal_buffer.create();
        normal_buffer.setUsagePattern( QGLBuffer::DynamicDraw );
        assert(normal_buffer.bind());
        normal_buffer.allocate(cm->cloud->points.size() * sizeof(float) * 3);

        for (int i = 0; i < (int)cm->cloud->size(); i++)
        {
            float data[3];
            data[0] = (cm->normals->at(i).data_n[0]*0.1);
            data[1] = (cm->normals->at(i).data_n[1]*0.1);
            data[2] = (cm->normals->at(i).data_n[2]*0.1);

            int offset = 3*sizeof(float)*i;
            normal_buffer.write(offset, reinterpret_cast<const void *> (data), sizeof(data));
        }

        normal_buffer.release();
        glError("normal buffer bad");
    }

    return true;
}
bool VizNormals::EndViz(CloudModel * cm, GLArea *){
    emit cm->layerList.setSelectMode(QAbstractItemView::MultiSelection);

    for(auto a: actionList){
        a->setChecked(false);
    }
    return true;
}

void VizNormals::paintLayer(int, CloudModel *, GLArea *){}

void VizNormals::paintGL(CloudModel * cm, GLArea * glarea){

    if(normal_buffer.isCreated()){

        assert(normal_shader.bind());
        float colour[4] = {1,0,0,1};
        glUniformMatrix4fv(normal_shader.uniformLocation("cameraToClipMatrix"), 1, GL_FALSE, glarea->camera.projectionMatrix().data());
        glUniformMatrix4fv(normal_shader.uniformLocation("modelToCameraMatrix"), 1, GL_FALSE, glarea->camera.modelviewMatrix().data());
        glUniform3fv(normal_shader.uniformLocation("lineColour"), 1, colour);
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT,  GL_NICEST);

        assert(normal_buffer.bind());
        normal_shader.enableAttributeArray("pointnormal");
        normal_shader.setAttributeBuffer("pointnormal", GL_FLOAT, 0, 3 );
        normal_buffer.release();

        assert(cm->point_buffer.bind());
        normal_shader.enableAttributeArray( "vertex" );
        normal_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
        cm->point_buffer.release();

        glDrawArrays(GL_POINTS, 0, cm->cloud->size());

        glError("Normals failed to draw");
        normal_buffer.release();
        normal_shader.release();
    }

}

QList<QAction *> VizNormals::actions() const{
    return actionList;
}
QString VizNormals::getEditToolDescription(QAction *){
    return "Info";
}

Q_EXPORT_PLUGIN2(pnp_vizNormals, VizNormals)
