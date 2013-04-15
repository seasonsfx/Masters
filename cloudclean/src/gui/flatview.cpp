#include "flatview.h"
#include <algorithm>
#include <QDebug>
#include <QMouseEvent>
#include <QApplication>
#include <QLabel>
#include <QMessageBox>
#include "commands/select.h"

FlatView::FlatView(QGLFormat & fmt, CloudList * cl,
                   LayerList * ll, QWidget *parent, QGLWidget *sharing)
    : QGLWidget(fmt, parent, sharing) {
    cl_ = cl;
    ll_ = ll;
    current_scale_ = 2;
    aspect_ratio_ = QVector2D(-1, -1);
    camera_.setIdentity();
    camera_(0,2) = -1;
    camera_(1,2) = -0.5;
    setMouseTracking(true);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));

    setContextMenuPolicy(Qt::CustomContextMenu);
}

void FlatView::setGLD(GLData * gld){
    gld_ = gld;
}


/*            height
 *
 *           ********
 *           ********
 *           ********
 *           ********
 *  width    ********
 *           ********
 *           ********
 *           ********
 *           ********
 */

const Eigen::Matrix3f FlatView::getCamera() {
    return camera_;
}

int binary_search(std::vector<int> A, int key) {
    int imin = 0;
    int imax = A.size();

    while (imax >= imin) {
        int imid = (imin+imax)/2;
        if (A[imid] < key)
            imin = imid + 1;
        else if (A[imid] > key)
            imax = imid - 1;
        else
            return imid;
    }
    return -1;
}

int FlatView::imageToCloudIdx(int x, int y){
    std::shared_ptr<PointCloud> pc = pc_.lock();
    if(x < 0 || x > pc->scan_width_){
        return -1;
    }
    else if(y < 0 || y > pc->scan_height_){
        return -1;
    }

    return cloud_idx_lookup_[x + y*pc->scan_width_];
}

// scan lines go from top to bottom & left to right

inline QPoint FlatView::cloudToImageCoord(int idx){
    std::shared_ptr<PointCloud> pc = pc_.lock();
    int i = pc->cloud_to_grid_map_[idx];
    int x = i/pc->scan_height_;
    int y = pc->scan_height_ - 1 - (i%pc->scan_height_);
    return QPoint(x, y);
}

void FlatView::setCloud(std::shared_ptr<PointCloud> new_pc) {
    if(new_pc == pc_.lock())
        return;

    if(!pc_.expired()){
        std::shared_ptr<PointCloud> old_pc = pc_.lock();
        disconnect(old_pc->ed_.get(), SIGNAL(flagUpdate()), this,
                   SLOT(update()));
        disconnect(old_pc->ed_.get(), SIGNAL(labelUpdate()), this,
                   SLOT(update()));
    }

    pc_ = new_pc;

    // Late event cause deletion possible? Seems to help?
    if(pc_.expired())
        return;

    std::shared_ptr<PointCloud> pc = pc_.lock();
    cloud_idx_lookup_.resize(pc->scan_width_*pc->scan_height_, -1);
    for(uint idx = 0 ; idx < pc->cloud_to_grid_map_.size(); idx++){
        QPoint p = cloudToImageCoord(idx);
        cloud_idx_lookup_[p.x() + p.y()*pc->scan_width_] = idx;
    }

    connect(pc->ed_.get(), SIGNAL(flagUpdate()), this, SLOT(update()));
    connect(pc->ed_.get(), SIGNAL(labelUpdate()), this, SLOT(update()));
    update();
}

void FlatView::mouseMoveEvent(QMouseEvent * event) {
    if(pc_.expired())
        return;

    if(event->buttons()){
        QVector2D dist(event->pos() - drag_start_pos);
        dist.setX(2.0f*dist.x()/width());
        dist.setY(2.0f*-dist.y()/height());
        dist += saved_offset_;

        camera_(0, 2) = dist.x();
        camera_(1, 2) = dist.y();

        update();
    }
}

void FlatView::mousePressEvent(QMouseEvent * event) {
    drag_start_pos = event->pos();
    saved_offset_ = QVector2D(camera_(0,2), camera_(1,2));
}

void FlatView::mouseReleaseEvent(QMouseEvent * event) {
    if (event->button() == Qt::RightButton)
    update();
}

void FlatView::wheelEvent(QWheelEvent * event) {
    auto sgn = [] (float val) {
        if(val > 0)
            return 1.0;
        else
            return -1.0;
    };

    float delta = sgn(event->delta())*1.0+(event->delta())/120.0;
    float s = 1.0f;

    // NDC translate
    Eigen::Matrix3f translate;
    translate.setIdentity();
    translate(0, 2) = 2.0f* ((event->x()/float(width())) - 0.5);
    translate(1, 2) = -2.0f* ((event->y()/float(height())) - 0.5);

    if(delta > 0 && current_scale_ < 100) {
        s = delta;
    }
    else if (delta < 0 && current_scale_ > 0.01) {
        s = 1.0/-delta;
    }

    Eigen::Matrix3f scale;
    scale.setIdentity();
    scale(0, 0) = s;
    scale(1, 1) = s;

    camera_ = translate * scale * translate.inverse() * camera_;

    current_scale_ *= s;
    update();
}

void FlatView::initializeGL() {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);

    //
    // Load shader program
    //
    bool succ = program_.addShaderFromSourceFile(
                QGLShader::Vertex, ":/flatview.vs.glsl"); CE();
    if (!succ)
        qWarning() << "Shader compile log:" << program_.log();
    succ = program_.addShaderFromSourceFile(
                QGLShader::Fragment, ":/flatview.fs.glsl"); CE();
    if (!succ)
        qWarning() << "Shader compile log:" << program_.log();
    succ = program_.addShaderFromSourceFile(
                QGLShader::Geometry, ":/flatview.gs.glsl"); CE();
    if (!succ)
        qWarning() << "Shader compile log:" << program_.log();

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
    //uni_width_ = program_.uniformLocation("width"); RC(uni_width_);
    uni_height_ = program_.uniformLocation("height"); RC(uni_height_);
    uni_select_color_ = program_.uniformLocation("select_color"); RC(uni_select_color_);
    uni_camera_ = program_.uniformLocation("camera"); RC(uni_camera_);
    program_.release();
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
    //
    // Generate vao
    //
    glGenVertexArrays(1, &vao_);
}


void FlatView::paintGL() {
    resizeGL(width(), height());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(pc_.expired()){
        qDebug() << "Nothing to paint on flatview";
        return;
    }

    std::shared_ptr<PointCloud> pc = pc_.lock();
    std::shared_ptr<CloudGLData> cd = gld_->cloudgldata_[pc];

    program_.bind(); CE();
    glUniformMatrix3fv(uni_camera_, 1, GL_FALSE, camera_.data()); CE();
    //glUniform1i(uni_width_, pc->scan_width_); CE();
    glUniform1i(uni_height_, pc->scan_height_); CE();
    glUniform1i(uni_sampler_, 0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, gld_->color_lookup_buffer_->bufferId()); CE();


    glBindVertexArray(vao_);CE();

    // Point intensity buffer
    IF_FAIL("buffer bind") = cd->point_buffer_->bind(); CE();
    glEnableVertexAttribArray(1); CE();
    int offset = sizeof(float)*3;
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float)*4,
                          reinterpret_cast<const void *>(offset)); CE();
    cd->point_buffer_->release(); CE();

    // Label buffer
    IF_FAIL("buffer bind") = cd->label_buffer_->bind(); CE();
    glEnableVertexAttribArray(2); CE(); CE();
    glVertexAttribIPointer(2, 1, GL_SHORT, 0, 0); CE();
    cd->label_buffer_->release(); CE();

    // Flag buffer
    IF_FAIL("buffer bind") = cd->flag_buffer_->bind(); CE();
    glEnableVertexAttribArray(3); CE();
    glVertexAttribIPointer(3, 1, GL_BYTE, 0, 0); CE();
    cd->flag_buffer_->release(); CE();

    // Grid buffer
    IF_FAIL("buffer bind") = cd->grid_buffer_->bind(); CE();
    glEnableVertexAttribArray(4); CE();
    glVertexAttribIPointer(4, 1, GL_INT, 0, 0); CE();
    cd->grid_buffer_->release(); CE();

    cd->draw(vao_); CE();

    glBindVertexArray(0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, 0); CE();

    program_.release();
}

void FlatView::resizeGL(int width, int height) {
    glViewport(0, 0, width, qMax(height, 1));

    if(pc_.expired())
        return;
    auto pc = pc_.lock();

    float war = width/float(height);
    float sar = pc->scan_width_/float(pc->scan_height_);

    float cfx = sar/war;
    float cfy = (1/sar)/(1/war);

    // if wider than scan
    if(war <  sar)
        aspect_ratio_ = QVector2D(1.0f/pc->scan_width_, 1.0/(cfx*pc->scan_height_));
    else
        aspect_ratio_ = QVector2D(1.0/(cfy*pc->scan_width_), 1.0f/pc->scan_height_);

    camera_(0, 0) = current_scale_*aspect_ratio_.x();
    camera_(1, 1) = current_scale_*aspect_ratio_.y();

    program_.bind(); CE();   
    glUniformMatrix3fv(uni_camera_, 1, GL_FALSE, camera_.data()); CE();
    program_.release(); CE();
}

void FlatView::contextMenu(const QPoint &pos) {

}
