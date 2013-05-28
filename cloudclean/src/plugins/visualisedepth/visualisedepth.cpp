#include "plugins/visualisedepth/visualisedepth.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QtWidgets>
#include <QScrollArea>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

QString VDepth::getName(){
    return "visualisedepth";
}

void VDepth::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;
    myaction_ = new QAction(QIcon(":/images/visualisedepth.png"), "Visualisedepth", 0);

    depth_widget_ = new QWidget(depth_widget_);
    tab_idx_ = core_->mw_->addTab(depth_widget_, "Depth map");

    QVBoxLayout * layout = new QVBoxLayout(depth_widget_);
    QScrollArea * scrollarea = new QScrollArea();
    scrollarea->setBackgroundRole(QPalette::Dark);
    layout->addWidget(scrollarea);
    image_container = new QLabel();
    scrollarea->setWidget(image_container);

    // Nonsense
    connect(myaction_,&QAction::triggered, [this] (bool on) {
        qDebug() << "Click!";
    });
    connect(myaction_, SIGNAL(triggered()), this, SLOT(myFunc()));
    mw_->toolbar_->addAction(myaction_);

}

void VDepth::cleanup(){
    mw_->toolbar_->removeAction(myaction_);
    mw_->removeTab(tab_idx_);
    delete myaction_;
    delete depth_widget_;
}

VDepth::~VDepth(){
    if(image != nullptr)
        delete image;
}

int gridToCloudIdx(int x, int y, std::shared_ptr<PointCloud> pc, int * lookup){
    if(x < 0 || x > pc->scan_width_)
        return -1;
    else if(y < 0 || y > pc->scan_height_)
        return -1;
    return lookup[x + y*pc->scan_width_];
}

void VDepth::myFunc(){
    qDebug() << "Myfunc";
    std::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;

    int size = cloud->scan_width_ * cloud->scan_height_;

    // translate grid idx to cloud idx
    std::vector<int> lookup(size, -1);

    // Create distance map
    std::vector<float> distmap(size, 0);

    float max_dist = 0;

    // Calculate distance from center of cloud
    for(int i = 0; i < cloud->size(); i++) {
        int grid_idx = cloud->cloud_to_grid_map_[i];
        lookup[grid_idx] = i; // construct map while we are at it
        pcl::PointXYZI & p = cloud->at(i);
        distmap[grid_idx] = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));

        if(distmap[i] > max_dist)
            max_dist = distmap[i];
    }

    ///////// Magic ///////////

    float sobel_x[9] = {
        1, 0, -1,
        2, 0, -2,
        1, 0, -1,
    };

    float sobel_y[9] = {
        1, 2, 1,
        0, 0, 0,
        -1, -2, -1,
    };

    std::vector<float> grad_mag(size, 0);

    int h = cloud->scan_width_;
    int w = cloud->scan_height_;

    auto convolve_op = [distmap, w, h] (int x, int y, float * filter) {

        float sum = 0.0f;

        for(int iy = -1; iy <= 1; iy++){
            for(int ix = -1; ix <= 1; ix++){
                // map pos
                int _x = ix + x;
                int _y = iy + y;

                // wraps around on edges
                if(_x == -1)
                    _x = w-1;
                else if(_x == w)
                    _x = 0;

                if(_y == -1)
                    _y = h-1;
                else if(_y == h)
                    _y = 0;

                // map index
                int i = _x + w * _y;
                // filter index
                int f = ix+1 + 3*(iy+1);

                sum += distmap[i] * filter[f];
            }
        }
        return sum;
    };

    float grad_max = 0.0f;
    float grad_min = FLT_MAX;

    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            float gx = convolve_op(x, y, sobel_x);
            float gy = convolve_op(x, y, sobel_y);
            grad_mag[x+y*w] = sqrt(gx*gx + gy*gy);

            if(grad_mag[x+y*w] > grad_max)
                grad_max = grad_mag[x+y*w];
            else if(grad_mag[x+y*w] < grad_min)
                grad_min = grad_mag[x+y*w];
        }
    }

    ///////// OUTPUT //////////

    if(image == nullptr)
        delete image;
    image = new QImage(cloud->scan_width_, cloud->scan_height_, QImage::Format_Indexed8);

    for(int i = 0; i < 256; i++) {
        image->setColor(i, qRgb(i, i, i));
    }

    // Write image
    for(int y = 0; y < cloud->scan_height_; y++){
        for(int x = 0; x < cloud->scan_width_; x++){
            int i = (cloud->scan_height_ -1 - y) + x * cloud->scan_height_;

            // Mask
            if(lookup[i] == -2) {
                image->setPixel(x, y, 0);
                continue;
            }

            //int intensity = 255 * (1 - distmap[i]/max_dist);
            float mag = grad_mag[i];
            int intensity = 255 * (1 - (mag - grad_min)/(grad_max - grad_min));

            if(intensity < 240) {
                intensity = 0;
            } else {
                intensity = 255;
            }

            image->setPixel(x, y, intensity);
        }
    }

    image_container->setPixmap(QPixmap::fromImage(*image));
    image_container->resize(image->size());

}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
