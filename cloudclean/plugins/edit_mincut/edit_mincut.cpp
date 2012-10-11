#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include <GL/glu.h>

#include <QIcon>
#include <QDebug>
#include <QResource>
#include <QAbstractItemView>
#include <QTime>

//#include <pcl/segmentation/min_cut_segmentation.h>

#include "edit_mincut.h"
#include "utilities.h"
#include "layer.h"
#include "glarea.h"
#include "cloudmodel.h"
#include "mincut.h"


EditPlugin::EditPlugin():
    vertex_buffer(QGLBuffer::IndexBuffer), edge_buffer(QGLBuffer::IndexBuffer)
{
    gdata_dirty = true;
    settings = new Settings();
    editSample = new QAction(QIcon(":/images/mincut.svg"), "Min cut", this);
    actionList << editSample;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

EditPlugin::~EditPlugin()
{
    delete settings;
}

void EditPlugin::paintGL(CloudModel * cm, GLArea * glarea){
    if(!settings->showGraph())
        return;

    if(!vertex_buffer.isCreated()){
        // Perpare shader
        assert(glarea->prepareShaderProgram(viz_shader,
                                            ":/shaders/points.vert",
                                            ":/shaders/points.frag",
                                            "" ) );

        if ( !viz_shader.bind() ) {
            qWarning() << "Could not bind shader program to context";
            assert(false);
        }
        viz_shader.enableAttributeArray( "vertex" );
        viz_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
        glUniformMatrix4fv(viz_shader.uniformLocation("modelToCameraMatrix"),
                           1, GL_FALSE, glarea->camera.modelviewMatrix().data());
        glUniformMatrix4fv(viz_shader.uniformLocation("cameraToClipMatrix"),
                           1, GL_FALSE, glarea->camera.projectionMatrix().data());
        viz_shader.release();
        edge_buffer.create();
        vertex_buffer.create();
    }


    // load data
    if(gdata_dirty){
        qDebug("loading gdata");
        gdata = seg.getGraphData();

        // create buffers

        edge_buffer.bind();
        size_t edge_size = 2 * sizeof(int);
        size_t edge_buffer_size = gdata->edges.size() * edge_size;
        edge_buffer.allocate(edge_buffer_size);
        for(int i = 0; i < gdata->edges.size(); i++){
            std::pair<int, int> & edge = gdata->edges[i];
            int data[] = {edge.first, edge.second};
            edge_buffer.write(i*edge_size, data, edge_size);
        }
        edge_buffer.release();


        vertex_buffer.bind();
        size_t vertex_buffer_size = gdata->vertices.size() * sizeof(int);
        vertex_buffer.allocate(vertex_buffer_size);
        for(int i = 0; i < gdata->vertices.size(); i++){
            int data = gdata->vertices[i];
            vertex_buffer.write(i*edge_size, &data, sizeof(int));
        }
        vertex_buffer.release();

        gdata_dirty = false;

    }

    // paint

    viz_shader.bind();
    glUniformMatrix4fv(viz_shader.uniformLocation("modelToCameraMatrix"),
                       1, GL_FALSE, glarea->camera.modelviewMatrix().data());
    glError("edit cut 1");
    glUniformMatrix4fv(viz_shader.uniformLocation("cameraToClipMatrix"),
                       1, GL_FALSE, glarea->camera.projectionMatrix().data());
    glError("edit cut 2");


    // enable attribur in shader
    viz_shader.enableAttributeArray( "vertex" );
    // bind the buffer to be used with this attribute
    cm->point_buffer.bind();
    // specify how to interpret buffer
    viz_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
    // this should be done for all attributes

    float colour[3] = {1,0,0};
    glUniform3fv(viz_shader.uniformLocation("elColour"), 1, colour);

    glLineWidth(1.0);
    edge_buffer.bind();
    glDrawElements(GL_LINES, gdata->edges.size(), GL_UNSIGNED_INT, 0);
    glError("edit cut 4");


    // Draw test line
    /*QGLBuffer myBuff;
    myBuff.create();
    myBuff.allocate(sizeof(float)*8);
    float data[] = {
            0, 0, 0, 0,
            1, 1, 1, 1
    };
    myBuff.write(0,data,sizeof(float)*8);
    myBuff.bind();
    viz_shader.enableAttributeArray( "vertex" );
    viz_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
    glDrawArrays(GL_LINES, 0, 2);
    myBuff.release();
    */



    // Draw edges


    /*for(int i = 0; i < gdata->edges.size(); i++){
        float colour[3] = {0,0,0};
        if(gdata->edge_label[i] == 0)
            colour[0] = 1; // red
        else if(gdata->edge_label[i] == 1)
            colour[2] = 1; // blue
        else if(gdata->edge_label[i] == 2)
            colour[1] = 1; // green

        glUniform3fv(viz_shader.uniformLocation("elColour"), 1, colour);
        //glLineWidth(gdata->edge_weights[i]);
        glLineWidth(1.0);
        glDrawRangeElements(GL_LINES, i*2, i*2+1, 2, GL_UNSIGNED_INT, 0);
        glError("edit cut 3");
    }
    */

    edge_buffer.release();
    cm->point_buffer.release();
    viz_shader.release();
    glError("edit cut 4");

    // paint all points
        // set up vertex buffer
        // copy to vertex buffer
        // draw
    // paint all edges
}

bool EditPlugin::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    return true;
}

bool EditPlugin::StartEdit(QAction *action, CloudModel *cm, GLArea *glarea){
    // Set up kdtree and octre if not set up yet

    cm->layerList.setSelectMode(QAbstractItemView::SingleSelection);

    dest_layer = -1;
    return true;
}
bool EditPlugin::EndEdit(CloudModel * cm, GLArea *){
    emit cm->layerList.setSelectMode(QAbstractItemView::MultiSelection);

    for(auto a: actionList){
        a->setChecked(false);
    }
    return true;
}

inline float clamp(float x, float a, float b)
{
    return x < a ? a : (x > b ? b : x);
}

inline float angularSimilarity(Eigen::Vector3f &a, Eigen::Vector3f &b){
    float cosine = a.dot(b) / (a.norm()*b.norm());
    cosine = clamp(cosine, 0.0f, 1.0f);
    float angularSimlarity = acos(cosine)/M_PI;
    return angularSimlarity;
}

void EditPlugin::fill(int x, int y, float radius, int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea){
    qDebug("running min cut");
    int index = glarea->pp->pick(x, y, source_idx);

    if(index == -1)
        return;

    pcl::PointXYZI p = cm->cloud->points[index];

    // Need to get rid of the -1 from the indices
    pcl::IndicesPtr source_indices(new std::vector<int>);
    for(int idx : cm->layerList.layers[source_idx].index){
        if(idx == -1)
            continue;
        source_indices->push_back(idx);
    }

    assert(source_indices->size() != 0 && "Source indices empty");

    seg.setInputCloud(cm->cloud);
    seg.setIndices(source_indices);
    //seg.setIndices(pcl::IndicesPtr(new std::vector<int>(cm->layerList.layers[source_idx].index)));

    pcl::PointCloud<pcl::PointXYZI>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZI> ());
    foreground_points->points.push_back(p);
    seg.setForegroundPoints (foreground_points);
    seg.setRadius (settings->radius());

    seg.setSigma (settings->sigma()); // I think this is density

    seg.setNumberOfNeighbours (settings->kConnectvity());
    seg.setSourceWeight (settings->sourceWeight());
    //seg.setHorisonalRadius(settings->horisontalRadius());

    std::vector <pcl::PointIndices> clusters;
    seg.extract (clusters);

    assert(clusters.size() != 0);

    gdata_dirty = true;

    // blank source & dest
    for(int i = 0; i < cm->cloud->points.size(); i++){
        cm->layerList.layers[source_idx].index[i] = -1;
        cm->layerList.layers[dest_idx].index[i] = -1;
    }


    // put clusters into layer
    for(int idx : clusters[0].indices){
        //qDebug("Cluster 0:  %d", idx);
        cm->layerList.layers[source_idx].index[idx] = idx;
    }

    for(int idx : clusters[1].indices){
        //qDebug("Cluster 1:  %d", idx);
        cm->layerList.layers[dest_idx].index[idx] = idx;
    }

    cm->layerList.layers[source_idx].copyToGPU();
    cm->layerList.layers[dest_idx].copyToGPU();

}

bool EditPlugin::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){

    return true;
}

bool EditPlugin::mouseMoveEvent (QMouseEvent *event, CloudModel *, GLArea * glarea){

    if(glarea->moved){

    }

    return true;
}

bool EditPlugin::mouseReleaseEvent(QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    if (!glarea->moved && event->button() == Qt::LeftButton){
        // Single selection switched on, on edit start

        int source_layer = -1;

        for(int i = 0; i < cm->layerList.layers.size(); i++){
            Layer & l = cm->layerList.layers[i];
            if(l.active && l.visible){
                source_layer = i;
                l.sync();;
                break;
            }
        }


        if(dest_layer == -1 || dest_layer >= cm->layerList.layers.size()){
            cm->layerList.newLayer();
            dest_layer = cm->layerList.layers.size()-1;
        }

        float radius = 1.0f;

        fill(event->x(), event->y(), radius, source_layer, dest_layer, cm, glarea);

    }

    return true;
}

QList<QAction *> EditPlugin::actions() const{
    return actionList;
}
QString EditPlugin::getEditToolDescription(QAction *){
    return "Info";
}

QWidget * EditPlugin::getSettingsWidget(QWidget *){
    return settings;
}

Q_EXPORT_PLUGIN2(pnp_editbrush, EditPlugin)
