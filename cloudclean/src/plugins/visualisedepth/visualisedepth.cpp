#include "plugins/visualisedepth/visualisedepth.h"
#include <iostream>
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QtWidgets>
#include <QScrollArea>
#include <QDoubleSpinBox>

#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_container.h>
#include <boost/serialization/shared_ptr.hpp>

#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "utilities/cv.h"
#include "utilities/utils.h"
#include "plugins/normalestimation/normalestimation.h"
#include "plugins/visualisedepth/utils.h"

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
}

void VDepth::initialize2(PluginManager * pm) {
    ne_ = pm->findPlugin<NormalEstimator>();
    if (ne_ == nullptr) {
        qDebug() << "Normal estimator plugin needed for normal viz";
        return;
    }

    function_idx_ = 0;
    layer_idx_ = -1;
    resolution_ = 0.5;

    // Set up viz tab
    depth_widget_ = new QWidget(0);
    tab_idx_ = core_->mw_->addTab(depth_widget_, "Feature visualisation");

    QVBoxLayout * tablayout = new QVBoxLayout(depth_widget_);
    QScrollArea * scrollarea = new QScrollArea();
    scrollarea->setBackgroundRole(QPalette::Dark);
    tablayout->addWidget(scrollarea);
    image_container_ = new QLabel();
    scrollarea->setWidget(image_container_);


    // set up settings
    is_enabled_ = false;
    enable_ = new QAction(QIcon(":/images/visualisedepth.png"), "Correlate and visualise", 0);
    enable_->setCheckable(true);

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    mw_->toolbar_->addAction(enable_);
    time = 0;
    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);
    mw_->tooloptions_->addWidget(settings_);

    // settings widgets
    layout->addWidget(new QLabel("Feature"));
    feature_cb_ = new QComboBox(settings_);
    layout->addWidget(feature_cb_);

    feature_cb_->addItem("Difference of normals", 0);
    feature_cb_->addItem("Fast point feature histograms", 1);
    feature_cb_->addItem("Curvature", 2);
    feature_cb_->addItem("Normal standard deviation", 3);
    feature_cb_->addItem("Eigen ratio", 4);
//    feature_cb_->addItem("Difference of normals", 5);
//    feature_cb_->addItem("Difference of normals", 6);
//    feature_cb_->addItem("Difference of normals", 7);

    connect(feature_cb_, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [this] (int idx){
        function_idx_ = feature_cb_->itemData(idx).toInt();
        qDebug() << "changed";
    });

    // round up functions
    functions_.push_back(std::bind(&VDepth::don_vis, this));
    functions_.push_back(std::bind(&VDepth::fpfh_correl, this));
    functions_.push_back(std::bind(&VDepth::curve_vis, this));
    functions_.push_back(std::bind(&VDepth::normal_stdev_vis, this));
    functions_.push_back(std::bind(&VDepth::eigen_ratio, this));

//    layout->addWidget(new QLabel("Correlate with layer:"));
//    layer_cb_ = new QComboBox(settings_);
//    layout->addWidget(layer_cb_);


    // Downsample settings
    QDoubleSpinBox * downsample_sb = new QDoubleSpinBox();
    downsample_sb->setAccelerated(true);
    downsample_sb->setMinimum(0.001);
    downsample_sb->setMaximum(1);
    downsample_sb->setValue(resolution_);
    connect(downsample_sb, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=] (double value){
        resolution_ = value;
    });

    layout->addWidget(new QLabel("Downsample resolution (meters)"));
    layout->addWidget(downsample_sb);


    // Run button
    QPushButton * run = new QPushButton("Correlate and visualise");
    connect(run, &QPushButton::clicked, [=] (){
        if(function_idx_ != -1){
            qDebug() << "call!";
            functions_[function_idx_]();
        }

    });
    layout->addWidget(run);

    layout->addStretch();

    connect(ll_, SIGNAL(dataChanged(QModelIndex,QModelIndex)), this, SLOT(layersModified()));
}

void VDepth::layersModified(){
    layer_cb_->clear();
    layer_cb_->addItem("Select", -1);

    layer_idx_ = -1;

    for(uint idx = 0; idx < ll_->getLayers().size(); idx++) {
        boost::shared_ptr<Layer> layer = ll_->getLayers()[idx];
        layer_cb_->addItem(layer->getName(), layer->getId());
    }
}

void VDepth::cleanup(){
    mw_->tooloptions_->removeWidget(settings_);
    mw_->toolbar_->removeAction(enable_);
    mw_->removeTab(tab_idx_);
    delete enable_;
    delete depth_widget_;
}

VDepth::~VDepth(){
    if(image_ != nullptr)
        delete image_;
}

void VDepth::enable() {
    if(is_enabled_){
        disable();
        return;
    }

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);
    emit enabling();
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void VDepth::disable(){
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = false;
}

int sum_calc(float * data, int vector_size, int size, float * sum, float * sum_of_squares, int stride = 0){
    stride = stride ? stride : vector_size;
    int skipped = 0;
    for(int j = 0; j < vector_size; j++){
        sum[j] = 0.0f;
        sum_of_squares[j] = 0.0f;
    }

    float val = 0;
    for(int i = 0; i < size; i++){
        for(int j = 0; j < vector_size; j++){
            val = data[i*stride + j];
            if(val != val){
                skipped++;
                continue;
            }
            sum[j] += val;
            sum_of_squares[j] += pow(val, 2);
        }
    }
    return skipped;
}

// uses y value for the last row of mat
void sum_product_mat_calc(float * y_data, float * x_data, int x_vector_size, int size, Eigen::MatrixXf & sum_product, int stride = 0){
    stride = stride ? stride : x_vector_size;
    sum_product.setIdentity();

    float val1 = 0, val2 = 0;
    for(int i = 0; i < size; i++){
        for(int r = 0; r < x_vector_size; r++){
            for(int c = r+1; c < x_vector_size; c++){
                val1 = (x_data[i*stride + r]);
                val2 = (x_data[i*stride + c]);

                if(val1 != val1 || val1 != val1){
                    continue;
                }

                sum_product(r, c) += val1*val2;
                sum_product(c, r) = sum_product(r, c);
            }

            sum_product(r, x_vector_size) += y_data[i] * x_data[i*stride + r];
            sum_product(x_vector_size, r) = sum_product(r, x_vector_size);
        }
    }
}

inline float correlate(float sx, float sxx, float sy, float syy, float sxy, float n){
    qDebug() << "sx: " << sx << "sxx: " << sxx << "sy: " << sy << "syy: " << syy << "sxy: " << sxy << "n: " << n;
    return (n*sxy-sx*sy)/sqrt((n*sxx-sx*sx) * (n*syy-sy*sy));
}

// blend the large segmentation to small!!!

// correlate binaryish with vector
Eigen::MatrixXf  multi_correlate(std::vector<float> & y_data, float * x_data, int x_vector_size, int size, int stride){
    std::vector<float> sum(x_vector_size + 1);
    std::vector<float> sum_of_squares(x_vector_size + 1);

    int skipped = sum_calc(x_data, x_vector_size, size, sum.data(), sum_of_squares.data(), stride);
    skipped += sum_calc(y_data.data(), 1, size, &sum[x_vector_size], &sum_of_squares[x_vector_size]);

    qDebug() << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1";
    qDebug() << "skipped: " << skipped << "size: " << size;

    Eigen::MatrixXf sum_product_mat(x_vector_size + 1, x_vector_size + 1);
    sum_product_mat_calc(y_data.data(), x_data, x_vector_size, size, sum_product_mat, stride);

    Eigen::MatrixXf correlation_mat(x_vector_size + 1, x_vector_size + 1);
    correlation_mat.setIdentity();

    for(int r = 0; r < x_vector_size + 1; r++){
        for(int c = r+1; c < x_vector_size + 1; c++){
            // qDebug() << r << c << ": " <<  sum[c] << sum_of_squares[c] << sum[r] << sum_of_squares[r] << sum_product_mat(r, c);
            correlation_mat(r, c) = correlate(sum[c], sum_of_squares[c], sum[r], sum_of_squares[r], sum_product_mat(r, c), size-skipped);
            correlation_mat(c, r) = correlation_mat(r, c);
        }
    }

    return correlation_mat;
}

std::vector<float> get_scaled_layer_mask(
        std::vector<uint16_t> labels_,
        std::set<uint16_t> segment_labels,
        std::vector<int> & big_to_small,
        int small_size){

    std::vector<float> mask(small_size, 0.0f);
    std::vector<int> mask_count(small_size, 0);

    for(uint big_idx = 0; big_idx < big_to_small.size(); big_idx++){
        int small_idx = big_to_small[big_idx];
        mask_count[small_idx]++;
        uint16_t label = labels_[big_idx];
        if(segment_labels.find(label) != segment_labels.end())
            mask[small_idx]++;
    }

    for(int i = 0; i < small_size; i++){
        mask[i] = mask_count[i] == 0 ? 0.0f : mask[i]/mask_count[i];
    }

    return mask;
}

int gridToCloudIdx(int x, int y, boost::shared_ptr<PointCloud> pc, int * lookup){
    if(x < 0 || x > pc->scan_width())
        return -1;
    else if(y < 0 || y > pc->scan_height())
        return -1;
    return lookup[x + y*pc->scan_width()];
}


void VDepth::computeCorrelation(float * data, int vector_size, int size, std::vector<int> & big_to_small, int stride){

    stride = stride ? stride : vector_size;

    if(ll_->getSelection().size() == 0)
        return;

    std::set<uint16_t> labels;
    for(boost::weak_ptr<Layer> wlayer: ll_->getSelection()){
        for(uint16_t label : wlayer.lock()->getLabelSet())
            labels.insert(label);
    }

    std::vector<float> layer = get_scaled_layer_mask(cl_->active_->labels_,
                          labels,
                          big_to_small,
                          size);

    Eigen::MatrixXf correlation_mat = multi_correlate(layer, data, vector_size, size, stride);
    Eigen::MatrixXf Rxx = correlation_mat.topLeftCorner(vector_size, vector_size);
    Eigen::VectorXf c = correlation_mat.block(0, vector_size, vector_size, 1);

    std::cout << correlation_mat << std::endl;

    float R = c.transpose() * (Rxx.inverse() * c);

    qDebug() << "R^2: " << R;
    qDebug() << "R: " << sqrt(R);

    Eigen::VectorXf tmp = (Rxx.inverse() * c);

    qDebug() << "Y -> X correlation <<<<<<<<<<<<<";
    std::cout << c << std::endl;
    qDebug() << "Coefs <<<<<<<<<<<<<";
    std::cout << tmp << std::endl;

}


void VDepth::drawFloats(boost::shared_ptr<const std::vector<float> > out_img, boost::shared_ptr<PointCloud> cloud){
    qDebug() << "DRAW!";
    // translates grid idx to cloud idx
    boost::shared_ptr<const std::vector<int>> lookup = cloud->gridToCloudMap();

    if(image_ != nullptr)
        delete image_;
    image_ = new QImage(cloud->scan_width(), cloud->scan_height(), QImage::Format_Indexed8);

    for(int i = 0; i < 256; i++) {
        image_->setColor(i, qRgb(i, i, i));
    }

    float min, max;
    min_max(*out_img, min, max);
    qDebug() << "Minmax" << min << max;

    // Draw image
    auto select = boost::make_shared<std::vector<int> >();
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

/*
            // Select
            if(lookup->at(i) != -1 && intensity > 100) {
                select->push_back(lookup->at(i));
            }

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

void VDepth::drawVector3f(boost::shared_ptr<const std::vector<Eigen::Vector3f> > out_img, boost::shared_ptr<PointCloud> cloud){

    // translates grid idx to cloud idx
    boost::shared_ptr<const std::vector<int>> lookup = cloud->gridToCloudMap();

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

// Kullback-Leibler distance

inline double KLDist(float * feature1, float * feature2, int size) {
    double kl = 0;

    for(int i = 0; i < size; i++){
        float p = feature1[i];
        float q = feature2[i];
        double tmp = p*log(p/q);

        if(tmp != tmp || tmp >= FLT_MAX || tmp <= FLT_MIN)
            continue;
        kl += tmp;
    }

    /*
    for(int i = 0; i < size; i++){
        std::cout << feature1[i] << "/" << feature2[i] << " ";
    }

    std::cout << std::endl << "feature:" << kl << std::endl;
    fflush(stdout);
    */
    return kl;
}

inline float EuclidianDist(float * feature1, float * feature2, int size) {
    float dist = 0;

    for(int i = 0; i < size; i++){
        dist += pow(feature1[i] - feature2[i], 2.0);
    }

    dist = sqrt(dist);

    if(dist != dist || dist >= FLT_MAX || dist <= FLT_MIN) {
        return 0;
    }

    return dist;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr zipNormals(
        pcl::PointCloud<pcl::PointXYZI> & cloud,
        pcl::PointCloud<pcl::Normal> & normals){

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr zipped (new pcl::PointCloud<pcl::PointXYZINormal>());
    zipped->resize(cloud.size());

    for(uint i = 0; i < cloud.size(); i ++){
        pcl::PointXYZI & p = cloud[i];
        pcl::Normal & n = normals[i];

        pcl::PointXYZINormal & pn = (*zipped)[i];
        pn.getNormalVector4fMap() = n.getNormalVector4fMap();
        pn.getVector4fMap() = p.getVector4fMap();
        pn.intensity = p.intensity;
    }

    return zipped;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr VDepth::gridDownsample(boost::shared_ptr<PointCloud> input, float resolution, std::vector<int>& sub_idxs) {
    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(input);

    // HACK: Make sure normals are not NaN or inf
    for (pcl::Normal & n : *normals) {
      if (!pcl::isFinite<pcl::Normal>(n)){
          n.normal_x = 0;
          n.normal_y = 0;
          n.normal_z = 1;
      }
    }


    // Zipper up normals and xyzi
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = zipNormals(*input, *normals);

    QTime t; t.start(); //qDebug() << "Timer started (Subsample)";
    /////////////////////////

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZINormal>());
    sub_idxs.resize(input->size(), 0);

    pcl::VoxelGrid<pcl::PointXYZINormal> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(resolution, resolution, resolution);
    sor.setDownsampleAllData (true);
    sor.setSaveLeafLayout(true);
    sor.filter(*output);


    for(uint i = 0; i < input->size(); i++) {
        pcl::PointXYZINormal &p = (*cloud)[i];
        int idx = sor.getCentroidIndex(p);
        sub_idxs[i] = idx;
    }


    /////////////////////////
    time = t.elapsed();

    return output;

}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr VDepth::downsample(
        boost::shared_ptr<PointCloud> input,
        float resolution,
        std::vector<int>& sub_idxs){

    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(input);

    // HACK: Make sure normals are not NaN or inf
    for (pcl::Normal & n : *normals) {
      if (!pcl::isFinite<pcl::Normal>(n)){
          n.getNormalVector3fMap() << 0, 0, 1;
      }
    }

    // Zipper up normals and xyzi
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = zipNormals(*input, *normals);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr output = octreeDownsample(cloud.get(), resolution_, sub_idxs);

    return output;

}


template<typename PointT, typename NormalT>
pcl::PointCloud<pcl::Normal>::Ptr don(
        pcl::PointCloud<PointT> & cloud,
        pcl::PointCloud<NormalT> & normals, float radius1 = 0.2,
        float radius2 = 0.5){

    pcl::PointCloud<pcl::Normal>::Ptr donormals;
    donormals.reset(new pcl::PointCloud<pcl::Normal>());
    donormals->resize(cloud.size());

    typename pcl::search::Search<PointT>::Ptr tree;
    tree.reset (new pcl::search::KdTree<PointT> (false));

    typename pcl::PointCloud<PointT>::ConstPtr cptr(&cloud, boost::serialization::null_deleter());
    tree->setInputCloud(cptr);

    std::vector<int> idxs;
    std::vector<float> sq_dists;

    auto avg_normal = [] (std::vector<int> idxs, pcl::PointCloud<NormalT> & normals) {
        pcl::Normal sum;
        for(int idx : idxs) {
            sum.getNormalVector3fMap() += normals[idx].getNormalVector3fMap();
        }
        sum.getNormalVector3fMap() /= idxs.size();
        return sum;
    };

    QTime t;
    t.start();

    for(uint idx = 0; idx < cloud.size(); idx++){
        std::vector<float> rads;
        rads.push_back(radius1);
        rads.push_back(radius2);
        std::vector<pcl::Normal> avgs;
        for(float rad : rads){
            idxs.clear();
            sq_dists.clear();
            tree->radiusSearch(idx, rad, idxs, sq_dists);
            idxs.push_back(idx);
            avgs.push_back(avg_normal(idxs, normals));
        }

        (*donormals)[idx].getNormalVector3fMap() = (avgs[0].getNormalVector3fMap() - avgs[1].getNormalVector3fMap())/2.0;
    }

    return donormals;
}

void VDepth::don_vis(){
    boost::shared_ptr<PointCloud> _cloud = core_->cl_->active_;
    if(_cloud == nullptr)
        return;

    int w = _cloud->scan_width();
    int h = _cloud->scan_height();

    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(_cloud);

    float res1 = resolution_;
    float res2 = resolution_*2;

    // Downsample
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr smaller_cloud;
    std::vector<int> sub_idxs;
    smaller_cloud = downsample(_cloud, res1, sub_idxs);

    pcl::PointCloud<pcl::Normal>::Ptr donormals = don(*smaller_cloud, *smaller_cloud, res1, res2);

    // Correlate
    computeCorrelation(reinterpret_cast<float*>(donormals->points.data()), 3, donormals->points.size(), sub_idxs, 4);


    // Draw
    pcl::PointCloud<pcl::Normal> big_donormals;
    map_small_to_big((*donormals).points, big_donormals.points, sub_idxs);

    const std::vector<int> & cloudtogrid = _cloud->cloudToGridMap();

    boost::shared_ptr<std::vector<Eigen::Vector3f> > grid = boost::make_shared<std::vector<Eigen::Vector3f> >(w*h, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    for(uint i = 0; i < normals->size(); i++){
        int grid_idx = cloudtogrid[i];
        (*grid)[grid_idx] = big_donormals[i].getNormalVector3fMap();
    }

    // Draw
    drawVector3f(grid, _cloud);
}

void VDepth::hist_vis(){
    boost::shared_ptr<PointCloud> _cloud = core_->cl_->active_;
    if(_cloud == nullptr)
        return;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr smaller_cloud;
    std::vector<int> sub_idxs;
    smaller_cloud = downsample(_cloud, resolution_, sub_idxs);


    int bins = 20;
    float radius = 0.05;
    int max_nn = 0;

    // Compute
    QTime t; t.start(); qDebug() << "Timer started (Intensity histogram)";
    boost::shared_ptr<std::vector<std::vector<float> > > hists = calcIntensityHist(*smaller_cloud, bins, radius, max_nn);
    qDebug() << "Intensity histogram" << t.elapsed() << " ms";

    // Copy to contigious structure
    std::vector<float> tmp(bins*smaller_cloud->size());
    for(int i = 0; i < smaller_cloud->size(); i++){
        for(int j = 0; j < bins; j++){
            tmp[i*bins+j] = (*hists)[i][j];
        }
    }

    // Correlate
    computeCorrelation(tmp.data(), bins, tmp.size(), sub_idxs);


    // Draw, TODO; Multidimentional draw
    boost::shared_ptr<std::vector<std::vector<float> > > hists2 = boost::make_shared<std::vector<std::vector<float> > >(_cloud->size());

    // map back to cloud
    for(uint i = 0; i < _cloud->size(); i++) {
        int idx = sub_idxs[i];
        (*hists2)[i] = (*hists)[idx];
    }

    hists.reset();

    boost::shared_ptr<const std::vector<int>> grid_to_cloud = _cloud->gridToCloudMap();

    int w = _cloud->scan_width();
    int h = _cloud->scan_height();

    // in the grid, subtract (x, y+1) from every (x, y)
    boost::shared_ptr<std::vector<float>> diffs = boost::make_shared<std::vector<float>>(w*h, 0.0f);

    //auto distfunc = EuclidianDist;
    auto distfunc = KLDist;

    /*
    // Lambda
    auto maxval = [] (float * array, int size) {
        float max = FLT_MIN;
        for(int i = 0; i < size; i++) {
            if(array[i] > max)
                max = array[i];
        }
        return max;
    };
    */

    const int feature_size = bins;

    for(int x = 1; x < w-1; x ++) {
        //qDebug() << "yes";
        for(int y = 1; y < h-1; y ++) {
            int center = (*grid_to_cloud)[x*h + y];
            int up = (*grid_to_cloud)[x*h + y-1];
            int down = (*grid_to_cloud)[x*h + y+1];
            int left = (*grid_to_cloud)[(x-1)*h + y];
            int right = (*grid_to_cloud)[(x+1)*h + y];


            (*diffs)[center] = 0;

            if(center == -1){
                continue;
            }

            // NB! assumed histograms are normalised

            if(up != -1 && down != -1){
                float * feature1 = &((*hists2)[up][0]);
                float * feature2 = &((*hists2)[down][0]);
                //float max1 = maxval(feature1, feature_size);
                //float max2 = maxval(feature2, feature_size);
                //(*diffs)[center] += fabs(max1 - max2);
                (*diffs)[center] += distfunc(feature1, feature2, feature_size);
            }

            if(left != -1 && right != -1){
                float * feature1 = &((*hists2)[left][0]);
                float * feature2 = &((*hists2)[right][0]);
                //float max1 = maxval(feature1, feature_size);
                //float max2 = maxval(feature2, feature_size);
                //(*diffs)[center] += fabs(max1 - max2);
                (*diffs)[center] += distfunc(feature1, feature2, feature_size);
            }

        }
    }

    boost::shared_ptr<const std::vector<float>> img = cloudToGrid(_cloud->cloudToGridMap(), w*h, diffs);

    drawFloats(img, _cloud);
    hists2.reset();
}

void VDepth::fpfh_correl(){
    boost::shared_ptr<PointCloud> _cloud = core_->cl_->active_;
    if(_cloud == nullptr)
        return;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr filt_cloud;
    std::vector<int> sub_idxs;
    filt_cloud = downsample(_cloud, resolution_, sub_idxs);

    // Compute
    QTime t; t.start(); qDebug() << "Timer started (FPFH)";
    pcl::FPFHEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(filt_cloud);
    fpfh.setInputNormals(filt_cloud);
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal>);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.20);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
    fpfh.compute(*fpfhs);
    qDebug() << "FPFH in " << t.elapsed() << " ms";

    // Correlate
    computeCorrelation(reinterpret_cast<float*>(fpfhs->points.data()), 33, fpfhs->points.size(), sub_idxs);
}

void VDepth::curve_vis(){
    boost::shared_ptr<PointCloud> _cloud = core_->cl_->active_;
    if(_cloud == nullptr)
        return;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr filt_cloud;
    std::vector<int> big_to_small;
    filt_cloud = downsample(_cloud, resolution_, big_to_small);

    // Compute
    QTime t; t.start(); qDebug() << "Timer started (Curvature)";

    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::PrincipalCurvatures> principal_curvatures_estimation;
    principal_curvatures_estimation.setInputCloud (filt_cloud);
    principal_curvatures_estimation.setInputNormals (filt_cloud);
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal>);
    principal_curvatures_estimation.setSearchMethod (tree);
    principal_curvatures_estimation.setRadiusSearch (0.5);
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principal_curvatures_estimation.compute (*principal_curvatures);
    qDebug() << "Curvature in " << t.elapsed() << " ms";

    // Correlate
    computeCorrelation(reinterpret_cast<float*>(principal_curvatures->points.data()), sizeof(pcl::PrincipalCurvatures)/sizeof(float), principal_curvatures->points.size(), big_to_small);

    // Draw
    int w = _cloud->scan_width();
    int h = _cloud->scan_height();

    boost::shared_ptr<std::vector<float>> grid = boost::make_shared<std::vector<float>>(w*h, 0.0f);
    const std::vector<int> & cloudtogrid = _cloud->cloudToGridMap();

    for(uint big_idx = 0; big_idx < _cloud->size(); big_idx++) {
        int small_idx = big_to_small[big_idx];
        int grid_idx = cloudtogrid[big_idx];
        pcl::PrincipalCurvatures & pc = (*principal_curvatures)[small_idx];
        (*grid)[grid_idx] = pc.pc1 + pc.pc2;
    }

    drawFloats(grid, _cloud);

}

void VDepth::normal_stdev_vis(){
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;

    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cloud);

    // Downsample and zipper up normals and xyzi
    std::vector<int> sub_idxs;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr smaller_cloud = zipNormals(cloud, normals);
    smaller_cloud = octreeDownsample(smaller_cloud.get(), resolution_, sub_idxs);


    float radius = 1, max_nn = 100;

    // Compute
    QTime t; t.start(); qDebug() << "Timer started (Normal stdev)";
    boost::shared_ptr<std::vector<float>> stdev = normal_stdev<pcl::PointXYZINormal, pcl::PointXYZINormal>(smaller_cloud, smaller_cloud, radius, max_nn);
    qDebug() << "Normal stdev in " << t.elapsed() << " ms";

    // Correlate
    computeCorrelation(stdev->data(), 1, stdev->size(), sub_idxs);

    // Draw
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    boost::shared_ptr<std::vector<float>> grid = boost::make_shared<std::vector<float>>(w*h, 0.0f);
    const std::vector<int> & cloudtogrid = cloud->cloudToGridMap();

    for(uint big_idx = 0; big_idx < cloud->size(); big_idx++) {
        int small_idx = sub_idxs[big_idx];
        int grid_idx = cloudtogrid[big_idx];
        (*grid)[grid_idx] = (*stdev)[small_idx];
    }
    drawFloats(grid, cloud);
}


void VDepth::dist_stdev(){
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;

    // Downsample and zipper up normals and xyzi
    std::vector<int> sub_idxs;
    pcl::PointCloud<pcl::PointXYZI>::Ptr smaller_cloud = octreeDownsample(cloud.get(), resolution_, sub_idxs);

    QTime t; t.start(); qDebug() << "Timer started (Dist stdev)";
    boost::shared_ptr<std::vector<float> > stdev = stdev_dist(smaller_cloud, 0.05f, 20, false);
    qDebug() << "Dist stdev in " << t.elapsed() << " ms";

    // Correlate
    computeCorrelation(stdev->data(), 1, stdev->size(), sub_idxs);

    // Draw
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    boost::shared_ptr<std::vector<float>> grid = boost::make_shared<std::vector<float>>(w*h, 0.0f);
    const std::vector<int> & cloudtogrid = cloud->cloudToGridMap();

    for(uint big_idx = 0; big_idx < cloud->size(); big_idx++) {
        int small_idx = sub_idxs[big_idx];
        int grid_idx = cloudtogrid[big_idx];
        (*grid)[grid_idx] = (*stdev)[small_idx];
    }

    drawFloats(grid, cloud);
}

void VDepth::sutract_lowfreq_noise(){
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    // Create distance map
    boost::shared_ptr<std::vector<float>> distmap = makeDistmap(cloud);

    //distmap = interpolate(distmap, w, h, 21);

    boost::shared_ptr<std::vector<float> > smooth_grad_image = convolve(distmap, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);

    boost::shared_ptr<std::vector<float>> highfreq = distmap;

    for(uint i = 0; i < distmap->size(); i++){
        (*highfreq)[i] = (*distmap)[i] - (*smooth_grad_image)[i];
    }

    drawFloats(highfreq, cloud);
}

void VDepth::eigen_ratio(){
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;

    // Downsample
    std::vector<int> sub_idxs;
    pcl::PointCloud<pcl::PointXYZI>::Ptr smaller_cloud = octreeDownsample(cloud.get(), resolution_, sub_idxs);

    // Compute
    QTime t; t.start(); qDebug() << "Timer started (PCA and speculaion)";
    boost::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(smaller_cloud.get(), 0.3f, 0);


    // Speculate
    boost::shared_ptr<std::vector<float>> likely_veg =
               boost::make_shared<std::vector<float>>(pca->size(), 0.0f);

    for(uint i = 0; i < pca->size(); i++) {
        Eigen::Vector3f eig = (*pca)[i];

        // Not enough neighbours
        if(eig[1] < eig[2]) {
            (*likely_veg)[i] = 0;
            continue;
        }

        /*
        float eig_sum = eig.sum();
        eig /= eig_sum;
        float fudge_factor = 5.0f;

        bool is_planar = eig[1] < 0.05 * fudge_factor || eig[2] < 0.01 * fudge_factor;

        if(!is_planar) {
            (*likely_veg)[i] = 0;
        } else {
            (*likely_veg)[i] = 1;
        }

        */

        float curvature;
        if(eig[0] > eig[2])
            curvature = eig[1] / (eig[0]);
        else
            curvature = 1;


        (*likely_veg)[i] = curvature;

    }

    qDebug() << "PCA in " << t.elapsed() << " ms";

    computeCorrelation(likely_veg->data(), 1, likely_veg->size(), sub_idxs);


    // Draw
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    boost::shared_ptr<std::vector<float>> likely_veg2 =
               boost::make_shared<std::vector<float>>(cloud->size(), 0.0f);


    for(uint i = 0; i < cloud->size(); i++) {
        uint subidx = sub_idxs[i];
        (*likely_veg2)[i] = (*likely_veg)[subidx];
    }


    boost::shared_ptr<const std::vector<float>> img = cloudToGrid(cloud->cloudToGridMap(), w*h, likely_veg2);

    drawFloats(img, cloud);
}

void VDepth::pca(){
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;

    std::vector<int> sub_idxs;
    pcl::PointCloud<pcl::PointXYZI>::Ptr smaller_cloud = octreeDownsample(cloud.get(), resolution_, sub_idxs);

    QTime t; t.start(); qDebug() << "Timer started (PCA)";
    boost::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(smaller_cloud.get(), 0.05f, 20);
    qDebug() << "PCA in " << t.elapsed() << " ms";


    computeCorrelation(reinterpret_cast<float *>(pca->data()), 3, pca->size(), sub_idxs, sizeof(Eigen::Vector3f)/sizeof(float));

//    // Draw
//    boost::shared_ptr<std::vector<Eigen::Vector3f> > grid = boost::make_shared<std::vector<Eigen::Vector3f> >(grid_to_cloud->size(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
//    for(int i = 0; i < grid_to_cloud->size(); i++) {
//        int idx = (*grid_to_cloud)[i];
//        if(idx != -1)
//            (*grid)[i] = (*pca)[idx];
//    }

//    drawVector3f(grid, cloud);


}

void VDepth::eigen_plane_consine_similarity(){
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;

    std::vector<int> sub_idxs;
    pcl::PointCloud<pcl::PointXYZI>::Ptr smaller_cloud = octreeDownsample(cloud.get(), resolution_, sub_idxs);

    QTime t; t.start(); qDebug() << "Timer started (eigen_plane_consine_similarity)";
    boost::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(smaller_cloud.get(), 0.05f, 20);

    boost::shared_ptr<std::vector<float>> plane_likelyhood =
               boost::make_shared<std::vector<float>>(pca->size(), 0.0f);

    Eigen::Vector3f ideal_plane(1.0f, 0.0f, 0.0f);
    ideal_plane.normalize();

    for(uint i = 0; i < pca->size(); i++) {
        Eigen::Vector3f & val = (*pca)[i];

        // Not enough neighbours
        if(val[1] < val[2]) {
            (*plane_likelyhood)[i] = 0;
            continue;
        }

        float similarity = cosine(val, ideal_plane);
        (*plane_likelyhood)[i] = similarity;
    }

    qDebug() << "eigen_plane_consine_similarity in " << t.elapsed() << " ms";

    // Correlate
    computeCorrelation(plane_likelyhood->data(), 1, plane_likelyhood->size(), sub_idxs);

    // Draw
    int h = cloud->scan_width();
    int w = cloud->scan_height();
/*
    boost::shared_ptr<std::vector<Eigen::Vector3f> > grid = boost::make_shared<std::vector<Eigen::Vector3f> >(grid_to_cloud->size(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    for(int i = 0; i < grid_to_cloud->size(); i++) {
        int idx = (*grid_to_cloud)[i];
        if(idx != -1)
            (*grid)[i] = (*pca)[idx];
    }

    drawVector3f(grid, cloud);
*/

    boost::shared_ptr<const std::vector<float>> img = cloudToGrid(cloud->cloudToGridMap(), w*h, plane_likelyhood);

    drawFloats(img, cloud);
}

void VDepth::sobel_erode(){
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    // Create distance map
    boost::shared_ptr<std::vector<float>> distmap = makeDistmap(cloud);

    boost::shared_ptr<std::vector<float> > grad_image = gradientImage(distmap, w, h);
    boost::shared_ptr<std::vector<float> > smooth_grad_image = convolve(grad_image, w, h, gaussian, 5);

    // Threshold && Erode

    const int strct[] = {
        0, 1, 0,
        1, 0, 1,
        0, 1, 0,
    };

    boost::shared_ptr<std::vector<float> > dilated_image =  morphology(
            smooth_grad_image,
            w, h, strct, 3, Morphology::ERODE,
            grad_image); // <-- reuse

    drawFloats(dilated_image, cloud);
}

void VDepth::sobel_blur(){
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    // Create distance map
    boost::shared_ptr<std::vector<float>> distmap = makeDistmap(cloud);

    boost::shared_ptr<std::vector<float> > smooth_grad_image = gradientImage(distmap, w, h);
    int blurs = 0;
    for(int i = 0; i < blurs; i++)
        smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);

    drawFloats(smooth_grad_image, cloud);
}

void VDepth::intensity_play() {
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;
    int h = cloud->scan_width();
    int w = cloud->scan_height();

    boost::shared_ptr<std::vector<float>> intensity = boost::make_shared<std::vector<float>>(cloud->size());

    // Create intensity cloud
    for(uint i = 0; i < intensity->size(); i++){
        (*intensity)[i] = (*cloud)[i].intensity;
    }

    boost::shared_ptr<std::vector<float>> img = cloudToGrid(cloud->cloudToGridMap(), w*h, intensity);


    boost::shared_ptr<std::vector<float> > smooth_grad_image = convolve(img, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);
    smooth_grad_image = convolve(smooth_grad_image, w, h, gaussian, 5);
/*
    boost::shared_ptr<std::vector<float>> highfreq = img;

    for(int i = 0; i < highfreq->size(); i++){
        (*highfreq)[i] = (*highfreq)[i] - (*smooth_grad_image)[i];
    }

    drawFloats(highfreq, cloud);
*/
    drawFloats(smooth_grad_image, cloud);

}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.visualisedepth")
