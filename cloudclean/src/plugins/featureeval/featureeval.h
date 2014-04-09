#ifndef FEATURE_EVAL_H
#define FEATURE_EVAL_H

#include <memory>
#include <functional>
#include <Eigen/Dense>
#include "glheaders.h"
#include "pluginsystem/iplugin.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <plugins/featureeval/export.h>

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
class QDebug;

class FE_API FeatureEval : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.featureeval" FILE "featureeval.json")
 public:
    QString getName();
    void initialize(Core * core);
    void initialize2(PluginManager * pm);
    void cleanup();
    ~FeatureEval();

    std::function<void()> getFunction(QString name);
    void setReportFuction(QDebug * dbg);


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

    void difference_of_normals();
    void intensity_histogram();
    void fast_point_feature_histogram();
    void curvature();
    void curve_diff_vis();
    void normal_standard_deviation();
    void distance_standard_deviation();
    void difference_of_gaussian_distances();
    void pca_eigen_value_ratio();
    void pca();
    void eigen_plane_consine_similarity();
    void sobel_erode();
    void sobel_blur();
    void blurred_intensity();

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
    double search_radius_;
    int max_nn_;

    bool visualise_on_;
    std::vector<std::function<void()>> functions_;
    std::map<QString, std::function<void()>> name_to_function_;

    QDebug * report_;
};

#endif  // FEATURE_EVAL_H
