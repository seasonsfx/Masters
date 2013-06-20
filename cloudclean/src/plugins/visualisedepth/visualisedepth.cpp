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
#include "plugins/normalestimation/normalestimation.h"

QString VDepth::getName(){
    return "visualisedepth";
}

void VDepth::initialize(Core *core){
    settings_ = nullptr;
    tab_idx_ = -1;

    image_container_ = nullptr;
    image_ = nullptr;

    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;
    myaction_ = new QAction(QIcon(":/images/visualisedepth.png"), "Visualisedepth", 0);

    depth_widget_ = new QWidget(0);
    tab_idx_ = core_->mw_->addTab(depth_widget_, "Depth map");

    QVBoxLayout * layout = new QVBoxLayout(depth_widget_);
    QScrollArea * scrollarea = new QScrollArea();
    scrollarea->setBackgroundRole(QPalette::Dark);
    layout->addWidget(scrollarea);
    image_container_ = new QLabel();
    scrollarea->setWidget(image_container_);

    // Nonsense
    connect(myaction_,&QAction::triggered, [this] (bool on) {
        qDebug() << "Click!";
    });
    connect(myaction_, SIGNAL(triggered()), this, SLOT(pca()));
    mw_->toolbar_->addAction(myaction_);

}

void VDepth::initialize2(PluginManager * pm) {
    ne_ = pm->findPlugin<NormalEstimator>();
    if (ne_ == nullptr) {
        qDebug() << "Normal estimator plugin needed for normal viz";
        return;
    }
}

void VDepth::cleanup(){
    mw_->toolbar_->removeAction(myaction_);
    mw_->removeTab(tab_idx_);
    delete myaction_;
    delete depth_widget_;
}

VDepth::~VDepth(){
    if(image_ != nullptr)
        delete image_;
}

int gridToCloudIdx(int x, int y, std::shared_ptr<PointCloud> pc, int * lookup){
    if(x < 0 || x > pc->scan_width())
        return -1;
    else if(y < 0 || y > pc->scan_height())
        return -1;
    return lookup[x + y*pc->scan_width()];
}

void VDepth::drawFloats(std::shared_ptr<const std::vector<float> > out_img, std::shared_ptr<PointCloud> cloud){
    qDebug() << "DRAW!";
    // translates grid idx to cloud idx
    std::shared_ptr<const std::vector<int>> lookup = cloud->gridToCloudMap();

    if(image_ != nullptr)
        delete image_;
    image_ = new QImage(cloud->scan_width(), cloud->scan_height(), QImage::Format_Indexed8);

    for(int i = 0; i < 256; i++) {
        image_->setColor(i, qRgb(i, i, i));
    }

    float min, max;
    minmax(*out_img, min, max);
    qDebug() << "Minmax" << min << max;

    // Draw image
    auto select = std::make_shared<std::vector<int> >();
    for(int y = 0; y < cloud->scan_height(); y++){
        for(int x = 0; x < cloud->scan_width(); x++){
            int i = (cloud->scan_height() -1 - y) + x * cloud->scan_height();

            // Mask disabled
            if(lookup->at(i) == -2) {
                image_->setPixel(x, y, 0);
                continue;
            }

            //int intensity = 255 * (1 - distmap[i]/max_dist);
            float mag = (*out_img)[i];
            //int intensity = 255 * (1 - (mag - min)/(max - min));

            int intensity = 255 * (mag - min)/(max - min);

            if(intensity > 255 || intensity < 0) {
                qDebug() << "Nope, sorry > 255 || < 0: " << mag;
                qDebug() << "Mag: " << mag;
                qDebug() << "Intensity" << intensity;
                return;
            }

            // Select
            if(lookup->at(i) != -1 && intensity > 100) {
                select->push_back(lookup->at(i));
            }
/*
            if(intensity < 40) {
                intensity = 0;
            } else {
                intensity = 255;
            }
*/
            image_->setPixel(x, y, intensity);
        }
    }

    qDebug() << "Done";

    core_->us_->beginMacro("Experiment");
    core_->us_->push(new Select(cloud, select));
    core_->us_->endMacro();
    qDebug() << "Done1";
    image_container_->setPixmap(QPixmap::fromImage(*image_));
    qDebug() << "Done1.1";
    image_container_->resize(image_->size());
    qDebug() << "Done 2";

}

void VDepth::drawVector3f(std::shared_ptr<const std::vector<Eigen::Vector3f> > out_img, std::shared_ptr<PointCloud> cloud){

    // translates grid idx to cloud idx
    std::shared_ptr<const std::vector<int>> lookup = cloud->gridToCloudMap();

    if(image_ != nullptr)
        delete image_;
    image_ = new QImage(cloud->scan_width(), cloud->scan_height(), QImage::Format_RGB32);

    // Draw image
    for(int y = 0; y < cloud->scan_height(); y++){
        for(int x = 0; x < cloud->scan_width(); x++){
            int i = (cloud->scan_height() -1 - y) + x * cloud->scan_height();

            // Mask disabled
            if(lookup->at(i) == -2) {
                image_->setPixel(x, y, 0);
                continue;
            }

            int r = (*out_img)[i][0] * 255;
            int g = (*out_img)[i][1] * 255;
            int b = (*out_img)[i][2] * 255;

            QRgb value = qRgb(r, g, b);

            image_->setPixel(x, y, value);
        }
    }


    image_container_->setPixmap(QPixmap::fromImage(*image_));
    image_container_->resize(image_->size());
}

void VDepth::normalnoise(){
    std::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cloud);

    // This is important so its cached and not recalculated
    std::shared_ptr<const std::vector<int>> grid_to_cloud = cloud->gridToCloudMap();

    std::shared_ptr<std::vector<float>> stdev = normal_stdev(cloud, normals, 1, 100);

    std::shared_ptr<const std::vector<float>> img = cloudToGrid(cloud->cloudToGridMap(), w*h, stdev);

    drawFloats(img, cloud);
}


void VDepth::dist_stdev(){
    std::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    // This is important so its cached and not recalculated
    std::shared_ptr<const std::vector<int>> grid_to_cloud = cloud->gridToCloudMap();

    std::shared_ptr<std::vector<float> > stdev = stdev_dist(cloud, 1.0f, 50, false);

    std::shared_ptr<const std::vector<float>> img = cloudToGrid(cloud->cloudToGridMap(), w*h, stdev);

    drawFloats(img, cloud);
}

void VDepth::sutract_lowfreq_noise(){
    std::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    // Create distance map
    std::shared_ptr<std::vector<float>> distmap = makeDistmap(cloud);

    //distmap = interpolate(distmap, w, h, 21);

    std::shared_ptr<std::vector<float> > smooth_grad_image = convolve(distmap, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);

    std::shared_ptr<std::vector<float>> highfreq = distmap;

    for(int i = 0; i < distmap->size(); i++){
        (*highfreq)[i] = (*distmap)[i] - (*smooth_grad_image)[i];
    }

    drawFloats(highfreq, cloud);
}

void VDepth::pca(){
    qDebug() << "Myfunc";
    std::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    // This is important so its cached and not recalculated
    std::shared_ptr<const std::vector<int>> grid_to_cloud = cloud->gridToCloudMap();

    std::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(cloud, 0.05f, 20);

    std::shared_ptr<std::vector<float>> plane_likelyhood =
               std::make_shared<std::vector<float>>(pca->size(), 0.0f);

    Eigen::Vector3f ideal_plane(1.0f, 0.0f, 0.0f);
    ideal_plane.normalize();

    for(int i = 0; i < pca->size(); i++) {
        Eigen::Vector3f & val = (*pca)[i];

        // Not enough neighbours
        if(val[1] < val[2]) {
            (*plane_likelyhood)[i] = 0;
            continue;
        }

        float similarity = cosine(val, ideal_plane);
        (*plane_likelyhood)[i] = similarity;
    }
/*
    std::shared_ptr<std::vector<Eigen::Vector3f> > grid = std::make_shared<std::vector<Eigen::Vector3f> >(grid_to_cloud->size(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    for(int i = 0; i < grid_to_cloud->size(); i++) {
        int idx = (*grid_to_cloud)[i];
        if(idx != -1)
            (*grid)[i] = (*pca)[idx];
    }

    drawVector3f(grid, cloud);
*/

    std::shared_ptr<const std::vector<float>> img = cloudToGrid(cloud->cloudToGridMap(), w*h, plane_likelyhood);

    drawFloats(img, cloud);
}

void VDepth::sobel_erode(){
    std::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    // Create distance map
    std::shared_ptr<std::vector<float>> distmap = makeDistmap(cloud);

    std::shared_ptr<std::vector<float> > grad_image = gradientImage(distmap, w, h);
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

    drawFloats(dilated_image, cloud);
}

void VDepth::myFunc(){
    qDebug() << "Myfunc";
    std::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    // This is important so its cached and not recalculated
    std::shared_ptr<const std::vector<int>> grid_to_cloud = cloud->gridToCloudMap();

    std::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(cloud, 1.0f, 50);

    std::shared_ptr<std::vector<Eigen::Vector3f> > grid = std::make_shared<std::vector<Eigen::Vector3f> >(grid_to_cloud->size(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    for(int i = 0; i < grid_to_cloud->size(); i++) {
        int idx = (*grid_to_cloud)[i];
        if(idx != -1)
            (*grid)[i] = (*pca)[idx];
    }

    //std::shared_ptr<std::vector<float> > stdev = stdev_depth(cloud, 1.0f);
    //std::shared_ptr<const std::vector<float>> img = cloudToGrid(cloud->cloudToGridMap(), w*h, stdev);


    /*
    // Create distance map
    std::shared_ptr<std::vector<float>> distmap = makeDistmap(cloud);
    //distmap = interpolate(distmap, w, h, 21);

    std::shared_ptr<std::vector<float> > smooth_grad_image = convolve(distmap, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);

    std::shared_ptr<std::vector<float>> highfreq = distmap;

    for(int i = 0; i < distmap->size(); i++){
        (*highfreq)[i] = (*distmap)[i] - (*smooth_grad_image)[i];
    }

/*
    //std::shared_ptr<std::vector<float> > grad_image = gradientImage(distmap, w, h, size);
    std::shared_ptr<std::vector<float> > int_image = interpolate(distmap, w, h, 50);
    std::shared_ptr<std::vector<float> > stdev_image = stdev(int_image, w, h, 5);


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

    //drawFloats(img, cloud);

    drawVector3f(grid, cloud);

}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
