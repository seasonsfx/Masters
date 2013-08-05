#ifndef VDEPTH_CUT_H
#define VDEPTH_CUT_H

#include <memory>
#include <Eigen/Dense>
#include "glheaders.h"
#include "pluginsystem/iplugin.h"
#include <pcl/point_types.h>

class PointCloud;
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class QLabel;
class NormalEstimator;

class VDepth : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "visualisedepth.json")
 public:
    QString getName();
    void initialize(Core * core);
    void initialize2(PluginManager * pm);
    void cleanup();
    ~VDepth();

 private:
    void drawFloats(std::shared_ptr<const std::vector<float> > out_img, std::shared_ptr<PointCloud> cloud);
    void drawVector3f(std::shared_ptr<const std::vector<Eigen::Vector3f> > out_img, std::shared_ptr<PointCloud> cloud);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr octreeDownsample(std::shared_ptr<PointCloud> input, float resolution, std::vector<int>& sub_idxs);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr gridDownsample(std::shared_ptr<PointCloud> input, float resolution, std::vector<int>& sub_idxs);

 private slots:
    void fpfh_vis();
    void normalnoise();
    void dist_stdev();
    void sutract_lowfreq_noise();
    void pca();
    void sobel_erode();
    void sobel_blur();
    void intensity_play();
    void myFunc();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * myaction_;
    QWidget * settings_;
    QWidget * depth_widget_;
    int tab_idx_;

    QLabel * image_container_;
    QImage * image_;

    NormalEstimator * ne_;
    float time = 0;

};

#endif  // VDEPTH_CUT_H
