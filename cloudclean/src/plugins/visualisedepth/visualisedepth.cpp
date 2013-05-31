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

    int h = cloud->scan_width_;
    int w = cloud->scan_height_;

    auto convolve_op = [w, h] (float * source, int x, int y, double * filter, int filter_size) {
        assert(filter_size%2 != 0);

        int start = -filter_size/2;
        int end = filter_size/2;

        float sum = 0.0f;

        for(int iy = start; iy <= end; iy++){
            for(int ix = start; ix <= end; ix++){
                // map pos
                int _x = ix + x;
                int _y = iy + y;

                // wraps around on edges
                if(_x < 0)
                    _x = w+_x;
                else if(_x > w-1)
                    _x = _x-w;

                if(_y < 0)
                    _y = h+_y;
                else if(_y > h-1)
                    _y = _y-h;

                // map index
                int i = _x + w * _y;
                // filter index
                int f = ix+1 + 3*(iy+1);

                sum += source[i] * filter[f];
            }
        }
        return sum;
    };

    double sobel_x[9] = {
        1, 0, -1,
        2, 0, -2,
        1, 0, -1,
    };

    double sobel_y[9] = {
        1, 2, 1,
        0, 0, 0,
        -1, -2, -1,
    };

    std::vector<float> grad_mag(size, 0);

    float grad_max = 0.0f;
    float grad_min = FLT_MAX;

    // Calculate the gradient magnitude
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            float gx = convolve_op(&distmap[0], x, y, sobel_x, 3);
            float gy = convolve_op(&distmap[0], x, y, sobel_y, 3);
            grad_mag[x+y*w] = sqrt(gx*gx + gy*gy);

            if(grad_mag[x+y*w] > grad_max)
                grad_max = grad_mag[x+y*w];
            else if(grad_mag[x+y*w] < grad_min)
                grad_min = grad_mag[x+y*w];
        }
    }


    double gaussian[25] = {
        0.00296901674395065, 0.013306209891014005, 0.02193823127971504, 0.013306209891014005, 0.00296901674395065,
        0.013306209891014005, 0.05963429543618023, 0.09832033134884507, 0.05963429543618023, 0.013306209891014005,
        0.02193823127971504, 0.09832033134884507, 0.16210282163712417, 0.09832033134884507, 0.02193823127971504,
        0.013306209891014005, 0.05963429543618023, 0.09832033134884507, 0.05963429543618023, 0.013306209891014005,
        0.00296901674395065, 0.013306209891014005, 0.02193823127971504, 0.013306209891014005, 0.00296901674395065,
    };

    std::vector<float> smooth_grad_mag(size, 0);


    // Apply gaussian
    for(int i = 0; i < 1; i++){
        grad_max = 0.0f;
        grad_min = FLT_MAX;
        for(int x = 0; x < w; x++){
            for(int y = 0; y < h; y++){
                float val = convolve_op(&grad_mag[0], x, y, gaussian, 5);
                smooth_grad_mag[x+y*w] = val;

                if(val > grad_max)
                    grad_max = val;
                else if(val < grad_min)
                    grad_min = val;
            }
        }
    }

    // Threshold && Erode
/*
    int strct = {
        0, 1, 0,
        1, 0, 1,
        0, 1, 0,
    };

    auto morph_op = [w, h] (float * source, float * dest, int x, int y, double * strct, int strct_size) {
        assert(filter_size%2 != 0);

        int start = -strct_size/2;
        int end = strct_size/2;
        int center_x, center_y;

        float sum = 0.0f;

        for(int iy = start; iy <= end; iy++){
            for(int ix = start; ix <= end; ix++){
                // map pos
                int _x = ix + x;
                int _y = iy + y;

                // wraps around on edges
                if(_x < 0)
                    _x = w+_x;
                else if(_x > w-1)
                    _x = _x-w;

                if(_y < 0)
                    _y = h+_y;
                else if(_y > h-1)
                    _y = _y-h;

                // map index
                int i = _x + w * _y;
                // filter index
                int f = ix+1 + 3*(iy+1);

                sum += source[i] * strct[f];
            }
        }
        return sum;
    };
*/
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
            float mag = smooth_grad_mag[i];
            int intensity = 255 * (1 - (mag - grad_min)/(grad_max - grad_min));

            if(intensity > 255) {
                qDebug() << "Nope, sorry > 255: " << mag;
                return;
            }

            if(intensity < 40) {
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
