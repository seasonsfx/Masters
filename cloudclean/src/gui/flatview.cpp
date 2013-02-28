#include "flatview.h"
#include <algorithm>
#include <QDebug>
#include <QMouseEvent>

FlatView::FlatView(QGLFormat & fmt, std::shared_ptr<CloudList> cl,
                   std::shared_ptr<LayerList> ll, QWidget *parent, QGLWidget *sharing)
    : QGLWidget(fmt, parent, sharing) {
    cl_ = cl;
    ll_ = ll;
    scale_ = 1.0f;
    aspect_ratio_ = QVector2D(-1, -1);
    offset_ = QPoint(0, 0);
    setMouseTracking(true); // Track mouse when up
}

void FlatView::setGLD(std::shared_ptr<GLData> gld){
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

inline int FlatView::imageToCloudIdx(int x, int y){
    std::shared_ptr<PointCloud> pc = pc_.lock();
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
    qDebug("set 2d cloudview");

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
    std::shared_ptr<PointCloud> pc = pc_.lock();
    cloud_idx_lookup_.resize(pc->scan_width_*pc->scan_height_, -1);
    for(int idx = 0 ; idx < pc->cloud_to_grid_map_.size(); idx++){
        QPoint p = cloudToImageCoord(idx);
        cloud_idx_lookup_[p.x() + p.y()*pc->scan_width_] = idx;
    }

    connect(pc->ed_.get(), SIGNAL(flagUpdate()), this, SLOT(update()));
    connect(pc->ed_.get(), SIGNAL(labelUpdate()), this, SLOT(update()));

    scale_ = 1.0f;
    offset_ = QPoint(0, 0);

    update();
}

void FlatView::mouseMoveEvent(QMouseEvent * event) {
    if(event->buttons() == Qt::LeftButton ){
        /*int idx = imageToCloudIdx(event->x(), event->y());
        if (idx != -1){
            std::shared_ptr<PointCloud> pc = pc_.lock();

            PointFlags & pf = pc->flags_[idx];
            pf = PointFlags((uint8_t(PointFlags::selected)) | uint8_t(pf));

            update();

            //emit labelUpdate();
            pc->ed_->emitflagUpdate();
        }
        */
    }
    else if(event->buttons() == Qt::RightButton){
         offset_ = saved_offset_ + event->pos() - drag_start_pos;
         update();
    }
}

void FlatView::mousePressEvent(QMouseEvent * event) {
    drag_start_pos = event->pos();
    saved_offset_ = offset_;
}

void FlatView::mouseReleaseEvent(QMouseEvent * event) {
    if (event->button() == Qt::RightButton)
    update();
}

void FlatView::wheelEvent(QWheelEvent * event) {
    float delta = (event->delta()/120.0f) * 1.1;

    if(delta > 0 && scale_ < 100){
        scale_ *= delta;
    }
    else if (delta < 0 && scale_ > 0.01){
        scale_ /= -delta;
    }
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
    uni_width_ = program_.uniformLocation("width"); RC(uni_width_);
    uni_height_ = program_.uniformLocation("height"); RC(uni_height_);
    uni_scale_ = program_.uniformLocation("scale"); RC(uni_scale_);
    uni_offset_ = program_.uniformLocation("offset"); RC(uni_offset_);
    uni_aspect_ratio_ = program_.uniformLocation("aspect_ratio"); RC(uni_aspect_ratio_);
    uni_select_color_ = program_.uniformLocation("select_color"); RC(uni_select_color_);
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
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(pc_.expired()){
        qDebug() << "Nothing to paint on flatview";
        return;
    }

    std::shared_ptr<PointCloud> pc = pc_.lock();
    std::shared_ptr<CloudGLData> cd = gld_->cloudgldata_[pc];

    program_.bind(); CE();

    glUniform1i(uni_width_, pc->scan_width_); CE();
    glUniform1i(uni_height_, pc->scan_height_); CE();
    glUniform1f(uni_scale_, scale_); CE();
    glUniform1i(uni_sampler_, 0); CE();
    glUniform2f(uni_offset_, offset_.x(), offset_.y());
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, gld_->color_lookup_buffer_->bufferId()); CE();


    glBindVertexArray(vao_);

    // Point intensity buffer
    cd->point_buffer_->bind(); CE();
    glEnableVertexAttribArray(1); CE();
    int offset = sizeof(float)*3;
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float)*4,
                          reinterpret_cast<const void *>(offset)); CE();
    cd->point_buffer_->release(); CE();

    // Label buffer
    cd->label_buffer_->bind(); CE();
    glEnableVertexAttribArray(2); CE(); CE();
    glVertexAttribIPointer(2, 1, GL_SHORT, 0, 0); CE();
    cd->label_buffer_->release(); CE();

    // Flag buffer
    cd->flag_buffer_->bind(); CE();
    glEnableVertexAttribArray(3); CE();
    glVertexAttribIPointer(3, 1, GL_BYTE, 0, 0); CE();
    cd->flag_buffer_->release(); CE();

    // Grid buffer
    cd->grid_buffer_->bind(); CE();
    glEnableVertexAttribArray(4); CE();
    glVertexAttribIPointer(4, 1, GL_INT, 0, 0); CE();
    cd->grid_buffer_->release(); CE();

    cd->draw(vao_);

    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_BUFFER, 0); CE();

    program_.release();
}

void FlatView::resizeGL(int width, int height) {
    glViewport(0, 0, width, qMax(height, 1));

    auto pc = pc_.lock();

    //float ar_screen = width / static_cast<float>(height);
    //float ar_scan = pc->scan_width_ / static_cast<float>(pc->scan_height_);

    float yscale = width/static_cast<float>(pc->scan_width_);
    float xscale = height/static_cast<float>(pc->scan_height_);

    if(yscale <  xscale){
        aspect_ratio_ = QVector2D(1, yscale*4);
    }
    else{
        aspect_ratio_ = QVector2D(xscale*4, 1);
    }

    /*
    if(ar_screen == 1.0f && ar_scan == 1.0f)
        aspect_ratio_ = QVector2D(1, 1);
    else if(ar_screen < 1.0f && ar_scan == 1.0f)
        aspect_ratio_ = QVector2D(xscale, 1);
    else if(ar_screen > 1.0f && ar_scan == 1.0f)
        aspect_ratio_ = QVector2D(1, yscale);
    else if(ar_screen == 1.0f && ar_scan < 1.0f)
        aspect_ratio_ = QVector2D(1, yscale);
    else if(ar_screen < 1.0f && ar_scan < 1.0f)
        aspect_ratio_ = QVector2D(xscale, 1);
    else if(ar_screen > 1.0f && ar_scan < 1.0f)
        aspect_ratio_ = QVector2D(1, yscale);
    else if(ar_screen == 1.0f && ar_scan > 1.0f)
        aspect_ratio_ = QVector2D(xscale, 1);
    else if(ar_screen < 1.0f && ar_scan > 1.0f)
        aspect_ratio_ = QVector2D(1, yscale);
    else if(ar_screen > 1.0f && ar_scan > 1.0f)
        aspect_ratio_ = QVector2D(1, yscale*2);
    */


    program_.bind(); CE();
    glUniform2f(uni_aspect_ratio_, aspect_ratio_.x(), aspect_ratio_.y()); CE();
    program_.release(); CE();

}
