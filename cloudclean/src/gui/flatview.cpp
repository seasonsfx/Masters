#include "flatview.h"
#include <algorithm>
#include <QDebug>
#include <QMouseEvent>

FlatView::FlatView(QGLFormat & fmt, std::shared_ptr<CloudList> cl,
                   std::shared_ptr<LayerList> ll, QWidget *parent, QGLWidget *sharing)
    : QGLWidget(fmt, parent, sharing) {
    cl_ = cl;
    ll_ = ll;
    img_dirty_ = true;
    max_intensity = 0;
    setMouseTracking(true); // Track mouse when up
}

void FlatView::setGLD(std::shared_ptr<GLData> gld){
    gld_ = gld;
}

void FlatView::paintEvent(QPaintEvent*) {
    if(img_dirty_){
        std::shared_ptr<PointCloud> pc = pc_.lock();
        updateImage();
        this->resize(pc->scan_width_, pc->scan_height_);
        update();
        qDebug("Reload image");
        img_dirty_ = false;
    }

    QPainter p(this);
    if(!img_.isNull()){
        p.drawImage(0, 0, img_);
    }
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
    //y = pc->scan_height_ - 1 - y;
    // Now in scan coordinates
    //int idx = pc->scan_height_ * x + y;
    //return binary_search(pc->cloud_to_grid_map_, idx);
}

// scan lines go from top to bottom & left to right

inline QPoint FlatView::cloudToImageCoord(int idx){
    std::shared_ptr<PointCloud> pc = pc_.lock();
    int i = pc->cloud_to_grid_map_[idx];
    int x = i/pc->scan_height_;
    int y = pc->scan_height_ - 1 - (i%pc->scan_height_);
    return QPoint(x, y);
}

void FlatView::updateImage(){
    std::shared_ptr<PointCloud> pc = pc_.lock();
    // Blank
    for(int y = 0 ; y < img_.height(); y++){
        for(int x = 0 ; x < img_.width(); x++){
            img_.setPixel(x, y, qRgb(0, 0, 0));
        }
    }

    for(int idx = 0 ; idx < pc->cloud_to_grid_map_.size(); idx++){
        pcl::PointXYZI & point = pc->at(idx);
        int intensity = 255*(point.intensity/max_intensity);

        QColor col(intensity, intensity, intensity);
        QColor sel(0, 0, 255);

        int label_id = pc->labels_[idx];
        std::shared_ptr<Layer> layer =
                ll_->layer_lookup_table_[label_id].lock();
        QColor & label_col = layer->color_;

        // Layer colors
        col.setRed(col.red()/255.0f*label_col.red());
        col.setGreen(col.green()/255.0f*label_col.green());
        col.setBlue(col.blue()/255.0f*label_col.blue());

        // Selection
        float mix = 0.5;
        float mix2 = 1.0 - mix;
        if(uint8_t(pc->flags_[idx]) & uint8_t(PointFlags::selected)
                && point.intensity == point.intensity){
            col.setRed(col.red()*mix + sel.red()*mix2);
            col.setGreen(col.green()*mix + sel.green()*mix2);
            col.setBlue(col.blue()*mix + sel.blue()*mix2);
        }

        QPoint p = cloudToImageCoord(idx);
        img_.setPixel(p.x(), p.y(), col.rgb());
        cloud_idx_lookup_[p.x() + p.y()*img_.width()] = idx;
    }
}

void FlatView::setCloud(std::shared_ptr<PointCloud> new_pc) {
    qDebug("set 2d cloudview");

    if(new_pc == pc_.lock())
        return;

    if(!pc_.expired()){
        std::shared_ptr<PointCloud> old_pc = pc_.lock();
        disconnect(old_pc->ed_.get(), SIGNAL(flagUpdate()), this,
                   SLOT(syncImage()));
        disconnect(old_pc->ed_.get(), SIGNAL(labelUpdate()), this,
                   SLOT(syncImage()));

    }
    pc_ = new_pc;
    std::shared_ptr<PointCloud> pc = pc_.lock();
    cloud_idx_lookup_.resize(pc->scan_width_*pc->scan_height_, -1);
    connect(pc->ed_.get(), SIGNAL(flagUpdate()), this, SLOT(syncImage()));
    connect(pc->ed_.get(), SIGNAL(labelUpdate()), this, SLOT(syncImage()));

    // find max intensity
    max_intensity = 0;
    for(int i = 0; i < pc->size(); i++){
        pcl::PointXYZI & point = pc->at(i);
        if(point.intensity > max_intensity)
            max_intensity = point.intensity;
    }

    img_ = QImage(pc->scan_width_, pc->scan_height_, QImage::Format_RGB32);
    updateImage();
    this->resize(pc->scan_width_, pc->scan_height_);
    update();
}

void FlatView::syncImage(){
    img_dirty_= true;
    if(isVisible())
        update();
}

void FlatView::mouseMoveEvent(QMouseEvent * event) {
    if(event->buttons()){
        int idx = imageToCloudIdx(event->x(), event->y());
        if (idx != -1){
            std::shared_ptr<PointCloud> pc = pc_.lock();

            PointFlags & pf = pc->flags_[idx];
            pf = PointFlags((uint8_t(PointFlags::selected)) | uint8_t(pf));


            pcl::PointXYZI & point = pc->at(idx);
            int intensity = 255*(point.intensity/max_intensity);
            QColor col(intensity, intensity, intensity);
            QColor sel(0, 0, 255);
            int label_id = pc->labels_[idx];
            std::shared_ptr<Layer> layer =
                    ll_->layer_lookup_table_[label_id].lock();
            QColor & label_col = layer->color_;

            // Layer colors
            col.setRed(col.red()/255.0f*label_col.red());
            col.setGreen(col.green()/255.0f*label_col.green());
            col.setBlue(col.blue()/255.0f*label_col.blue());

            // Selection
            float mix = 0.5;
            float mix2 = 1.0 - mix;
            if(uint8_t(pc->flags_[idx]) & uint8_t(PointFlags::selected)
                    && point.intensity == point.intensity){
                col.setRed(col.red()*mix + sel.red()*mix2);
                col.setGreen(col.green()*mix + sel.green()*mix2);
                col.setBlue(col.blue()*mix + sel.blue()*mix2);
            }

            img_.setPixel(event->x(), event->y(), col.rgb());
            update();

            //emit labelUpdate();
            disconnect(pc->ed_.get(), SIGNAL(flagUpdate()), this, SLOT(syncImage()));
            pc->ed_->emitflagUpdate();
            connect(pc->ed_.get(), SIGNAL(flagUpdate()), this, SLOT(syncImage()));
        }
    }
}

void FlatView::mousePressEvent(QMouseEvent * event) {

}

void FlatView::mouseReleaseEvent(QMouseEvent * event) {
    if (event->button() == Qt::RightButton)
    update();
}

void GLWidget::initializeGL() {
    glClearColor(0.8, 0.8, 0.8, 1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);

    //
    // Load shader program
    //
    bool succ = program_.addShaderFromSourceFile(
                QGLShader::Vertex, ":/points.vert"); CE();
    qWarning() << program_.log();
    if (!succ) qWarning() << "Shader compile log:" << program_.log();
    succ = program_.addShaderFromSourceFile(
                QGLShader::Fragment, ":/points.frag"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_.log();
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
    glPointSize(point_render_size_);

    // Generate vao
    glGenVertexArrays(1, &vao_);
}


void FlatView::paintGL() {
    std::shared_ptr<PointCloud> pc = pc_.lock();
    std::shared_ptr<CloudGLData> cd = gld_->cloudgldata_[pc];

    // Make sure the labels are updates
    // Make sure nothing has changed

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program_.bind(); CE();

    glUniform1i(uni_width_, pc->scan_with_); CE();
    glUniform1i(uni_height_, pc->scan_height_); CE();

    glUniform1i(uni_sampler_, 0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, gld_->color_lookup_buffer_->bufferId()); CE();

    // TODO(Rickert): Cloud position in world space

    glBindVertexArray(vao_);

    // Grid buffer
    cd->grid_buffer_->bind(); CE();
    // Grid pos buffer
    grid_buffer_->bind(); CE();
    glEnableVertexAttribArray(1); CE();
    glVertexAttribIPointer(1, 1, GL_INT, 0, 0); CE();
    grid_buffer_->release(); CE();
    cd->grid_buffer_->release(); CE();

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

    cd->draw(vao_);
    glBindVertexArray(0);

    glBindTexture(GL_TEXTURE_BUFFER, 0); CE();

    program_.release();
}

void GLWidget::resizeGL(int width, int height) {
    camera_.setAspect(width / static_cast<float>(height));
    glViewport(0, 0, width, qMax(height, 1));
}
