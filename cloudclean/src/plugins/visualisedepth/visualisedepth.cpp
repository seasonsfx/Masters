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
#include "plugins/visualisedepth/utils.h"

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

    // translates grid idx to cloud idx
    std::shared_ptr<std::vector<int>> lookup = makeLookup(cloud);

    // Create distance map
    std::shared_ptr<std::vector<float>> distmap = makeDistmap(cloud);


    int h = cloud->scan_width_;
    int w = cloud->scan_height_;

    std::shared_ptr<std::vector<float> > grad_image = gradientImage(distmap, w, h, size);
    std::shared_ptr<std::vector<float> > stdev_image = stdev(grad_image, w, h, 5);

/*
    std::shared_ptr<std::vector<float> > grad_image = gradientImage(distmap, w, h, size);
    std::shared_ptr<std::vector<float> > smooth_grad_image = convolve(grad_image, w, h, gaussian, 5);

    // Threshold && Erode

    const int strct[] = {
        0, 1, 0,
        1, 0, 1,
        0, 1, 0,
    };

    std::shared_ptr<std::vector<float> > dilated_image =  morphology(
            smooth_grad_image,
            w, h, strct, 3, Morphology::ERODE,
            grad_image); // <-- reuse
*/


    ///////// OUTPUT //////////

    std::shared_ptr<std::vector<float> > out_img = stdev_image;

    if(image == nullptr)
        delete image;
    image = new QImage(cloud->scan_width_, cloud->scan_height_, QImage::Format_Indexed8);

    for(int i = 0; i < 256; i++) {
        image->setColor(i, qRgb(i, i, i));
    }

    float min, max;
    minmax(*out_img, min, max);
    qDebug() << "Minmax" << min << max;

    // Draw image
    auto select = std::make_shared<std::vector<int> >();
    for(int y = 0; y < cloud->scan_height_; y++){
        for(int x = 0; x < cloud->scan_width_; x++){
            int i = (cloud->scan_height_ -1 - y) + x * cloud->scan_height_;

            // Mask disabled
            if(lookup->at(i) == -2) {
                image->setPixel(x, y, 0);
                continue;
            }

            //int intensity = 255 * (1 - distmap[i]/max_dist);
            float mag = (*out_img)[i];
            int intensity = 255 * (1 - (mag - min)/(max - min));

            if(intensity > 255) {
                qDebug() << "Nope, sorry > 255: " << mag;
                return;
            }

            // Select
            if(lookup->at(i) != -1 && intensity < 230) {
                select->push_back(lookup->at(i));
            }
/*
            if(intensity < 40) {
                intensity = 0;
            } else {
                intensity = 255;
            }
*/
            image->setPixel(x, y, intensity);
        }
    }

    core_->us_->beginMacro("Experiment");
    core_->us_->push(new Select(cloud, select));
    core_->us_->endMacro();

    image_container->setPixmap(QPixmap::fromImage(*image));
    image_container->resize(image->size());

}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
