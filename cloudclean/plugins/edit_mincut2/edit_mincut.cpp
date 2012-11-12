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
    for(unsigned int i = 0; i < edges.size(); i++){
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
    glError("vertex mem 1");
    buff.write(0, &vertices[0], vertex_buffer_size);
    glError("vertex mem 2");
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

    // Perpare edge shader
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

        source_edge_weight_buffer.create();
        sink_edge_weight_buffer.create();
        bridge_edge_weight_buffer.create();

        // Create textures ids
        glGenTextures(3,textures);
    }


    // Perpare vertex shader
    if(!viz_shader2.isLinked()){
        assert(glarea->prepareShaderProgram(viz_shader2,
                                            ":/shader/points.vert",
                                            ":/shader/points.frag",
                                            "" ) );
        if ( !viz_shader2.bind() ) {
            qWarning() << "Could not bind shader program to context";
            assert(false);
        }
        viz_shader2.enableAttributeArray( "vertex" );
        viz_shader2.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
        glUniformMatrix4fv(viz_shader2.uniformLocation("modelToCameraMatrix"),
                           1, GL_FALSE, glarea->camera.modelviewMatrix().data());
        glUniformMatrix4fv(viz_shader2.uniformLocation("cameraToClipMatrix"),
                           1, GL_FALSE, glarea->camera.projectionMatrix().data());
        viz_shader2.release();

        source_vertex_buffer.create();
        sink_vertex_buffer.create();
    }


    // load data
    if(gdata_dirty){
        qDebug("loading gdata");
        gdata = seg.getGraphData();

        qDebug("Source edges: %lu", gdata->source_edges.size());
        qDebug("Sink edges: %lu", gdata->sink_edges.size());

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

    glEnable(GL_DEPTH_TEST);

    ///////////// paint edges

    viz_shader.bind();
    glUniformMatrix4fv(viz_shader.uniformLocation("modelToCameraMatrix"),
                       1, GL_FALSE, glarea->camera.modelviewMatrix().data());
    glUniformMatrix4fv(viz_shader.uniformLocation("cameraToClipMatrix"),
                       1, GL_FALSE, glarea->camera.projectionMatrix().data());
    glUniform1f(viz_shader.uniformLocation("max_line_width"), settings->edgeWidth());

    // enable attribur in shader
    viz_shader.enableAttributeArray( "vertex" );
    // bind the buffer to be used with this attribute
    cm->point_buffer.bind();
    // specify how to interpret buffer
    viz_shader.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );
    // this should be done for all attributes

    float colour [4] = {1,0,0,1}; // Red
    glUniform3fv(viz_shader.uniformLocation("elColour"), 1, colour);
    source_edge_buffer.bind();
    glBindTexture(GL_TEXTURE_BUFFER, textures [0]);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, source_edge_weight_buffer.bufferId());
    glDrawElements(GL_LINES, gdata->source_edges.size()*2, GL_UNSIGNED_INT, 0);
    glBindTexture(GL_TEXTURE_BUFFER, 0);

    colour [0] = 0; colour [2] = 1; // Blue
    glUniform3fv(viz_shader.uniformLocation("elColour"), 1, colour);
    sink_edge_buffer.bind();
    glBindTexture(GL_TEXTURE_BUFFER, textures [0]);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, sink_edge_weight_buffer.bufferId());
    glDrawElements(GL_LINES, gdata->sink_edges.size()*2, GL_UNSIGNED_INT, 0);
    glBindTexture(GL_TEXTURE_BUFFER, 0);

    colour [2] = 0; colour [1] = 1; // Green
    glUniform3fv(viz_shader.uniformLocation("elColour"), 1, colour);
    bridge_edge_buffer.bind();
    glBindTexture(GL_TEXTURE_BUFFER, textures [0]);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, bridge_edge_weight_buffer.bufferId());
    glDrawElements(GL_LINES, gdata->bridge_edges.size()*2, GL_UNSIGNED_INT, 0);
    glBindTexture(GL_TEXTURE_BUFFER, 0);

    bridge_edge_buffer.release();
    cm->point_buffer.release();
    viz_shader.release();
    glError("edit cut 5");


    ////////////////// Draw vertices //////////////////////////////

    viz_shader2.bind();
    glUniformMatrix4fv(viz_shader2.uniformLocation("modelToCameraMatrix"),
                       1, GL_FALSE, glarea->camera.modelviewMatrix().data());
    glUniformMatrix4fv(viz_shader2.uniformLocation("cameraToClipMatrix"),
                       1, GL_FALSE, glarea->camera.projectionMatrix().data());

    // enable attribur in shader
    viz_shader2.enableAttributeArray( "vertex" );
    cm->point_buffer.bind();
    viz_shader2.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );

    glPointSize(settings->vertexSize());

    colour [0] = 1.0f; colour [1] = 0.0f; // Red
    glUniform4fv(viz_shader2.uniformLocation("colour"), 1, colour);
    glError("Setting uniform is an issue");
    source_vertex_buffer.bind();
    glError("binding");
    glDrawElements(GL_POINTS, gdata->source_vertices.size(), GL_UNSIGNED_INT, 0);
    glError("draw");
    source_vertex_buffer.release();

    colour [0] = 0.0f; colour [2] = 1.0f; // Blue
    glUniform4fv(viz_shader2.uniformLocation("colour"), 1, colour);
    glError("Setting uniform is an issue 2");
    sink_vertex_buffer.bind();
    glError("binding");
    assert(gdata->sink_vertices.size() == sink_vertex_buffer.size()/sizeof(int));
    //assert(gdata->sink_vertices.size() != 0);
    glDrawElements(GL_POINTS, gdata->sink_vertices.size(), GL_UNSIGNED_INT, 0);
    glError("draw");
    sink_vertex_buffer.release();
    cm->point_buffer.release();

    // Draw center line with same shader /////////////////////////

    int far = 100;
    Eigen::Vector3f center_line_origin(seg.subcloud_center_.x(), seg.subcloud_center_.y(), 0);
    Eigen::Vector3f center_line_dir(0, 0, 1);

    Eigen::Vector3f start = center_line_origin;
    Eigen::Vector3f end = center_line_dir * far;

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    if(!temp_buffer.isCreated())
        temp_buffer.create();

    temp_buffer.bind();
    viz_shader2.setAttributeBuffer( "vertex", GL_FLOAT, 0, 4 );

    temp_buffer.allocate(sizeof(float)*4*(seg.polygon_.size()+1)*2);

    float one = 1;
    float data[8] = {0};

    memcpy(&data[0], start.data(), sizeof(float)*3); memcpy(&data[3], &one, sizeof(float));
    memcpy(&data[4], end.data(), sizeof(float)*3); memcpy(&data[7], &one, sizeof(float));

    temp_buffer.write(0, data, sizeof(float)*8);

    colour [0] = 1.0f; colour [1] = 0.08f; colour [2] = 0.57f; // pink
    colour [3] = 0.8f; // opacity
    glUniform4fv(viz_shader2.uniformLocation("colour"), 1, colour);
    glLineWidth(8);
    glDrawArrays(GL_LINES, 0, 2);

    glError("Draw line Issues");

    // draw polygon sides

    //qDebug("Start draw fan");

    // for each pair of points
    for(unsigned int i = 0 ; i < seg.polygon_.size() + 1; i ++){
        int idx = i % seg.polygon_.size();

        Eigen::Vector3f & point = seg.polygon_[idx];

        Eigen::Vector3f start = seg.cam_origin_;
        Eigen::Vector3f end = (point - seg.cam_origin_).normalized() * far;

        memcpy(&data[0], start.data(), sizeof(float)*3); memcpy(&data[3], &one, sizeof(float));
        memcpy(&data[4], end.data(), sizeof(float)*3); memcpy(&data[7], &one, sizeof(float));

        //qDebug("(%f, %f, %f, %f), (%f, %f, %f, %f)", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );

        temp_buffer.write(i*sizeof(float)*8, data, sizeof(float)*8);

    }
    glError("Write Issues");

    colour [0] = 0.0f; colour [1] = 0.0f; colour [2] = 0.0f; // black
    glUniform4fv(viz_shader2.uniformLocation("colour"), 1, colour);

    glEnable (GL_BLEND);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, (seg.polygon_.size() + 1) * 2);
    glError("Draw Issues");

    temp_buffer.release();

    glError("Issues");

    viz_shader2.release();


}

bool EditPlugin::mouseDoubleClickEvent  (QMouseEvent *, CloudModel *, GLArea *){
    lasso_active = !lasso_active;
    return true;
}

bool EditPlugin::StartEdit(QAction *, CloudModel * cm, GLArea * glarea){
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


bool EditPlugin::mousePressEvent  (QMouseEvent *, CloudModel *, GLArea *){

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
            for(unsigned int i = 0; i < cm->layerList.layers.size(); i++){
                Layer & l = cm->layerList.layers[i];
                if(l.active && l.visible){
                    source_layer = i;
                    l.sync();
                    break;
                }
            }

            // If the destination layer does not exist or has been deleted, fix it
            if(dest_layer == -1 || dest_layer >= (int)cm->layerList.layers.size()){
                cm->layerList.newLayer();
                dest_layer = cm->layerList.layers.size()-1;
            }

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

Eigen::Vector2f centroid(const std::vector<Eigen::Vector2f> polygon){
    float x = 0, y = 0;

    float signedArea = 0;

    for(unsigned int i = 0; i < polygon.size(); i++){
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

Eigen::Vector3f meanPoint(pcl::IndicesPtr & indices, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    Eigen::Vector3f avg_point;

    for(int idx : *indices){
        pcl::PointXYZI & p = cloud->at(idx);
        Eigen::Vector3f tmp(p.x, p.y, p.z);
        avg_point += tmp;
    }
    avg_point /= indices->size();
    return avg_point;
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


    // find the centroid of the polygon
    //Eigen::Vector2f centroid2d = centroid(polygon);

    // Find the 3D polygon
    float z = 1.0f;
    std::vector<Eigen::Vector3f> poly3d = unProjectPolygonNDC(
                                    glarea->camera.projectionMatrix().matrix(),
                                    glarea->camera.modelviewMatrix().matrix(),
                                    z,
                                    polygon);

    /*Eigen::Vector3f centroid3d = unProjectNDC(
                glarea->camera.projectionMatrix().matrix(),
                glarea->camera.modelviewMatrix().matrix(),
                z,
                centroid2d);
    */

    Eigen::Vector3f center = meanPoint(inside_lasso_indices, cm->cloud);

    seg.setBoundingPolygon(poly3d, center);
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
