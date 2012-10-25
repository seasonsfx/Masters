#define GL3_PROTOTYPES
#include <../../external/gl3.h>
#include <GL/glu.h>

#include <QIcon>
#include <QDebug>
#include <QResource>
#include <QAbstractItemView>
#include <QTime>

#include "edit_mincut.h"
#include "utilities.h"
#include "layer.h"
#include "glarea.h"
#include "cloudmodel.h"
#include "mincut.h"


EditPlugin::EditPlugin():
    source_vertex_buffer(QGLBuffer::IndexBuffer),
    sink_vertex_buffer(QGLBuffer::IndexBuffer),
    source_edge_buffer(QGLBuffer::IndexBuffer),
    sink_edge_buffer(QGLBuffer::IndexBuffer),
    bridge_edge_buffer(QGLBuffer::IndexBuffer)
{
    gdata_dirty = true;
    lasso_active = false;
    normalised_mouse_loc = Eigen::Vector2f(0,0);
    settings = new Settings();
    editSample = new QAction(QIcon(":/images/mincut.svg"), "Min cut (lasso)", this);
    actionList << editSample;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);

}

EditPlugin::~EditPlugin()
{
    delete settings;
}

inline void edgesToBuffer(std::vector<std::pair<int, int> > & edges,
                     QGLBuffer & buff){

    // create buffers
    buff.bind();
    size_t edge_size = 2 * sizeof(int);
    size_t edge_buffer_size = edges.size() * edge_size;
    buff.allocate(edge_buffer_size);
    for(int i = 0; i < edges.size(); i++){
        std::pair<int, int> & edge = edges [i];
        int data [] = {edge.first, edge.second};
        buff.write(i*edge_size, data, edge_size);
    }
    buff.release();
}

inline void verticesToBuffer(std::vector<int> & vertices,
                     QGLBuffer & buff){

    // create buffers
    buff.bind();
    size_t vertex_buffer_size = vertices.size() * sizeof(int);
    buff.allocate(vertex_buffer_size);
    buff.write(0, &vertices[0], vertex_buffer_size);
    buff.release();
}

inline void weightsToBuffer(std::vector<float> & weights,
                     QGLBuffer & buff){

    // create buffers
    buff.bind();
    size_t vertex_buffer_size = weights.size() * sizeof(float);
    buff.allocate(vertex_buffer_size);

    buff.write(0, &weights[0], vertex_buffer_size);
    buff.release();
}


void EditPlugin::paintGL(CloudModel * cm, GLArea * glarea){
    lasso.drawLasso(normalised_mouse_loc, glarea);

    if(!settings->showGraph())
            return;

        // Perpare shader
        if(!viz_shader.isLinked()){
            assert(glarea->prepareShaderProgram(viz_shader,
                                                ":/shader/graph.vert",
                                                ":/shader/graph.frag",
                                                ":/shader/graph.geom" ) );
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
            glUniform1i(viz_shader.uniformLocation("sampler"), 0);
            glUniform1f(viz_shader.uniformLocation("max_line_width"), 0.005f);
            viz_shader.release();

            source_edge_buffer.create();
            sink_edge_buffer.create();
            bridge_edge_buffer.create();
            source_vertex_buffer.create();
            sink_vertex_buffer.create();

            source_edge_weight_buffer.create();
            sink_edge_weight_buffer.create();
            bridge_edge_weight_buffer.create();

            // Create textures ids
            glGenTextures(3,textures);

        }

        // load data
        if(gdata_dirty){
            qDebug("loading gdata");
            gdata = seg.getGraphData();

            qDebug("Source edges: %d", gdata->source_edges.size());
            qDebug("Sink edges: %d", gdata->sink_edges.size());

            edgesToBuffer(gdata->source_edges, source_edge_buffer);
            edgesToBuffer(gdata->sink_edges, sink_edge_buffer);
            edgesToBuffer(gdata->bridge_edges, bridge_edge_buffer);

            verticesToBuffer(gdata->source_vertices, source_vertex_buffer);
            verticesToBuffer(gdata->sink_vertices, sink_vertex_buffer);

            weightsToBuffer(gdata->source_edge_weights, source_edge_weight_buffer);
            weightsToBuffer(gdata->sink_edge_weights, sink_edge_weight_buffer);
            weightsToBuffer(gdata->bridge_edge_weights, bridge_edge_weight_buffer);

            gdata_dirty = false;

        }

        // paint

        viz_shader.bind();
        glUniformMatrix4fv(viz_shader.uniformLocation("modelToCameraMatrix"),
                           1, GL_FALSE, glarea->camera.modelviewMatrix().data());
        glError("edit cut 1");
        glUniformMatrix4fv(viz_shader.uniformLocation("cameraToClipMatrix"),
                           1, GL_FALSE, glarea->camera.projectionMatrix().data());
        glUniform1f(viz_shader.uniformLocation("max_line_width"), settings->edgeWidth());
        glError("edit cut 2");

        // enable attribur in shader
        viz_shader.enableAttributeArray( "vertex" );
        // bind the buffer to be used with this attribute
        cm->point_buffer.bind();
        // specify how to interpret buffer
        viz_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
        // this should be done for all attributes

        float colour [3] = {1,0,0}; // Red
        glUniform3fv(viz_shader.uniformLocation("elColour"), 1, colour);
        source_edge_buffer.bind();
        glBindTexture(GL_TEXTURE_BUFFER, textures [0]);
        glError("edit cut 000");
        glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, source_edge_weight_buffer.bufferId());
        glError("edit cut 111");
        //glBindTexture(GL_TEXTURE_BUFFER, 0);
        //qDebug("Size %d", source_edge_buffer.size());
        glDrawElements(GL_LINES, gdata->source_edges.size()*2, GL_UNSIGNED_INT, 0);

        colour [0] = 0; colour [1] = 1; // Green
        glUniform3fv(viz_shader.uniformLocation("elColour"), 1, colour);
        sink_edge_buffer.bind();
        glBindTexture(GL_TEXTURE_BUFFER, textures [0]);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, sink_edge_weight_buffer.bufferId());
        //glBindTexture(GL_TEXTURE_BUFFER, 0);
        glDrawElements(GL_LINES, gdata->sink_edges.size()*2, GL_UNSIGNED_INT, 0);

        colour [1] = 0; colour [2] = 1; // Blue
        glUniform3fv(viz_shader.uniformLocation("elColour"), 1, colour);
        bridge_edge_buffer.bind();
        glBindTexture(GL_TEXTURE_BUFFER, textures [0]);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, bridge_edge_weight_buffer.bufferId());
        //glBindTexture(GL_TEXTURE_BUFFER, 0);
        glDrawElements(GL_LINES, gdata->bridge_edges.size()*2, GL_UNSIGNED_INT, 0);

        bridge_edge_buffer.release();
        cm->point_buffer.release();
        viz_shader.release();
        glError("edit cut 5");


        viz_shader.release();


        // Draw vertices
        glarea->point_shader.bind();
        glUniformMatrix4fv(glarea->point_shader.uniformLocation("modelToCameraMatrix"),
                           1, GL_FALSE, glarea->camera.modelviewMatrix().data());
        glUniformMatrix4fv(glarea->point_shader.uniformLocation("cameraToClipMatrix"),
                           1, GL_FALSE, glarea->camera.projectionMatrix().data());
glError("274");
        glarea->point_shader.enableAttributeArray( "vertex" );
        glarea->point_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );

        cm->point_buffer.bind();


        glPointSize(settings->vertexSize());

        colour [0] = 1; colour [2] = 0; // Red
        glUniform3fv(glarea->point_shader.uniformLocation("layerColour"), 1, colour);
        source_vertex_buffer.bind();
        qDebug("Size: %d, created = %s", source_vertex_buffer.size(),
               source_vertex_buffer.isCreated() ? "true" : "false");


        for(int i = 0; i < gdata->source_vertices.size(); i++){
            qDebug("i = %d & size = %d", i, cm->cloud->size());
            qDebug("Val cpu: %d", gdata->source_vertices[i]);
            int num;
            source_vertex_buffer.read(i*sizeof(int), &num, sizeof(int));
            qDebug("Val gpu: %d", num);
        }

        glDrawElements(GL_POINTS, gdata->source_vertices.size(), GL_UNSIGNED_INT, 0);
        source_vertex_buffer.release();

        colour [0] = 0; colour [1] = 1; // Green
        glUniform3fv(glarea->point_shader.uniformLocation("layerColour"), 1, colour);
        sink_vertex_buffer.bind();
        glDrawElements(GL_POINTS, gdata->sink_vertices.size(), GL_UNSIGNED_INT, 0);
        sink_vertex_buffer.release();

        cm->point_buffer.release();
        glarea->point_shader.release();



}

bool EditPlugin::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel * cm, GLArea * glarea){
    lasso_active = !lasso_active;
    return true;
}

bool EditPlugin::StartEdit(QAction *action, CloudModel *cm, GLArea *glarea){
    cm->layerList.setSelectMode(QAbstractItemView::SingleSelection);
    dest_layer = -1;

    connect(settings, SIGNAL(repaint()), glarea, SLOT(repaint()), Qt::UniqueConnection);
    return true;
}
bool EditPlugin::EndEdit(CloudModel * cm, GLArea *){
    emit cm->layerList.setSelectMode(QAbstractItemView::MultiSelection);

    for(auto a: actionList){
        a->setChecked(false);
    }
    lasso.clear();
    return true;
}


bool EditPlugin::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){

    return true;
}

bool EditPlugin::mouseMoveEvent (QMouseEvent *event, CloudModel *, GLArea * glarea){

    /// Save the last known normalised mouse positionb
    if(glarea->moved){
        normalised_mouse_loc = glarea->normalized_mouse(
                    event->x(), event->y());
        if(lasso_active)
            glarea->updateGL();
    }

    return true;
}

bool EditPlugin::mouseReleaseEvent(QMouseEvent *event, CloudModel * cm, GLArea * glarea){

    if (!glarea->moved && event->button() == Qt::LeftButton){
        // Single selection switched on, on edit start

        if(!lasso_active && lasso.getPolygon().size() > 3){

            int source_layer = -1;

            // Sets source layer as the one selected
            for(int i = 0; i < cm->layerList.layers.size(); i++){
                Layer & l = cm->layerList.layers[i];
                if(l.active && l.visible){
                    source_layer = i;
                    l.sync();
                    break;
                }
            }

            // If the destination layer does not exist or has been deleted, fix it
            if(dest_layer == -1 || dest_layer >= cm->layerList.layers.size()){
                cm->layerList.newLayer();
                dest_layer = cm->layerList.layers.size()-1;
            }


            qDebug("segmenting????????????????????");
            segment(source_layer, dest_layer, cm, glarea);
            lasso.clear();
        }
        else if(lasso_active){
            lasso.addPoint(this->normalised_mouse_loc);
        }

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

Eigen::Vector3f unProjectNDC(const Eigen::Matrix4f & proj,
                             const Eigen::Matrix4f & mv,
                             const float z,
                             const Eigen::Vector2f p){

    Eigen::Vector4f screen_p(p.x(), p.y(), z, 1.0f);
    Eigen::Vector4f unprojected_p = (proj * mv).inverse() * screen_p;
    unprojected_p /= unprojected_p[3];
    assert(unprojected_p.w() != 0.0f);
    return unprojected_p.head<3>();
}

// Unproject from 2D NDC space
std::vector<Eigen::Vector3f> unProjectPolygonNDC(
                                const Eigen::Matrix4f & proj,
                                const Eigen::Matrix4f & mv,
                                const float z,
                                const std::vector<Eigen::Vector2f> inpoly){

    std::vector<Eigen::Vector3f> unprojected_polygon;

    for(Eigen::Vector2f p: inpoly){
        Eigen::Vector4f screen_p(p.x(), p.y(), z, 1.0f);
        // Unproject
        Eigen::Vector4f unprojected_p = (proj * mv).inverse() * screen_p;
        unprojected_p /= unprojected_p[3];
        assert(unprojected_p.w() != 0.0f);
        unprojected_polygon.push_back(unprojected_p.head<3>());
    }

    return unprojected_polygon;
}

Eigen::Vector2f centoid(const std::vector<Eigen::Vector2f> polygon){
    float x = 0, y = 0;

    float signedArea = 0;

    for(int i = 0; i < polygon.size(); i++){
        Eigen::Vector2f p1 = polygon[i];
        Eigen::Vector2f p2 = polygon[(i+1)%polygon.size()];

        float a = p1.x()*p2.y() - p2.x()-p1.y();
        signedArea += a;
        x += (p1.x() + p2.x())*a;
        y += (p1.y() + p2.y())*a;
    }

    x/=(6*signedArea);
    y/=(6*signedArea);
    return Eigen::Vector2f(x,y);
}

void EditPlugin::segment(int source_idx, int dest_idx, CloudModel *cm, GLArea * glarea){


    // Fetch point indices inside the lasso
    Eigen::Matrix4f ndc_trans = glarea->camera.projectionMatrix().matrix() *
            glarea->camera.modelviewMatrix().matrix();

    std::vector<int> & source_layer = cm->layerList.layers[source_idx].index;
    std::vector<int> & dest_layer = cm->layerList.layers[dest_idx].index;

    // Get points inside lasso
    pcl::IndicesPtr inside_lasso_indices(new std::vector<int>);
    lasso.getIndices(ndc_trans, &*cm->cloud, source_layer, *inside_lasso_indices);

    seg.setInputCloud(cm->cloud);
    seg.setIndices(inside_lasso_indices);

    auto polygon = lasso.getPolygon();


    // find the centoid of the polygon
    Eigen::Vector2f centoid2d = centoid(polygon);

    // Find the 3D polygon
    float z = 1.0f;
    std::vector<Eigen::Vector3f> poly3d = unProjectPolygonNDC(
                                    glarea->camera.projectionMatrix().matrix(),
                                    glarea->camera.modelviewMatrix().matrix(),
                                    z,
                                    polygon);

    Eigen::Vector3f centoid3d = unProjectNDC(
                glarea->camera.projectionMatrix().matrix(),
                glarea->camera.modelviewMatrix().matrix(),
                z,
                centoid2d);


    seg.setBoundingPolygon(poly3d, centoid3d);
    seg.setCameraOrigin(glarea->camera.position());

    seg.setSigma (settings->sigma()); // Density me thinks
                                      // try set this dynamically
    seg.setRadius (settings->radius());
    seg.setNumberOfNeighbours (settings->kConnectvity());
    seg.setSourceWeight (settings->sourceWeight());

    std::vector <pcl::PointIndices> clusters;
    seg.extract (clusters);

    assert(clusters.size() != 0);
    gdata_dirty = true;

    // copy the segmented region from the source to the dest

    // put clusters into layer
    /*for(int idx : clusters[0].indices){
        //source_layer[idx] = idx;
        //dest_layer[idx] = -1;
    }
*/
    for(int idx : clusters[1].indices){
        dest_layer[idx] = idx;
        source_layer[idx] = -1;
    }


    cm->layerList.layers[source_idx].copyToGPU();
    cm->layerList.layers[dest_idx].copyToGPU();

}

Q_EXPORT_PLUGIN2(pnp_editbrush, EditPlugin)
