#ifndef VDEPTH_CUT_H
#define VDEPTH_CUT_H

#include <memory>
#include <functional>
#include <Eigen/Dense>
#include "glheaders.h"
#include "pluginsystem/iplugin.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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
class QComboBox;

class VDepth : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.visualisedepth" FILE "visualisedepth.json")
 public:
    QString getName();
    void initialize(Core * core);
    void initialize2(PluginManager * pm);
    void cleanup();
    ~VDepth();

 private:
    void computeCorrelation(float * data, int vector_size, int size, std::vector<int> & big_to_small, int stride = 0);
    void drawFloats(boost::shared_ptr<const std::vector<float> > out_img, boost::shared_ptr<PointCloud> cloud);
    void drawVector3f(boost::shared_ptr<const std::vector<Eigen::Vector3f> > out_img, boost::shared_ptr<PointCloud> cloud);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr downsample(
            boost::shared_ptr<PointCloud> input,
            float resolution, std::vector<int> & sub_idxs);


    pcl::PointCloud<pcl::PointXYZINormal>::Ptr gridDownsample(boost::shared_ptr<PointCloud> input, float resolution, std::vector<int>& sub_idxs);

signals:
  void enabling();

 private slots:
    void layersModified();
    void enable();
    void disable();

    void don_vis();
    void hist_vis();
    void fpfh_correl();
    void curve_vis();
    void curve_diff_vis();
    void normal_stdev_vis();
    void dist_stdev();
    void sutract_lowfreq_noise();
    void eigen_ratio();
    void pca();
    void eigen_plane_consine_similarity();
    void sobel_erode();
    void sobel_blur();
    void intensity_play();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * enable_;
    QWidget * settings_;
    bool is_enabled_;

    QWidget * depth_widget_;
    int tab_idx_;

    QLabel * image_container_;
    QImage * image_;

    NormalEstimator * ne_;
    float time;

    QComboBox * feature_cb_;
    QComboBox * layer_cb_;
    int function_idx_;
    int layer_idx_;

    double resolution_;

    std::vector<std::function<void()>> functions_;
};

#endif  // VDEPTH_CUT_H
