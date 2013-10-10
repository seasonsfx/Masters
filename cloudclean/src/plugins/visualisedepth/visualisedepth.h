#ifndef VDEPTH_CUT_H
#define VDEPTH_CUT_H

#include <memory>
#include <Eigen/Dense>
#include "glheaders.h"
#include "pluginsystem/iplugin.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_container.h>
#include <boost/serialization/shared_ptr.hpp>

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

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr octreeDownsample(
            std::shared_ptr<PointCloud> input,
            float resolution, std::vector<int> & sub_idxs);


    // Templated version
    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr octreeDownsample(
            pcl::PointCloud<PointT> * input,
            float resolution,
            std::vector<int> & sub_idxs) {

        size_t data_items = sizeof(PointT)/sizeof(float);

        typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>());
        sub_idxs.resize(input->size(), 0);

        typename pcl::PointCloud<PointT>::Ptr ptr(input, boost::serialization::null_deleter());

        typename pcl::octree::OctreePointCloud<PointT> octree1(resolution);
        octree1.setInputCloud (ptr);
        octree1.addPointsFromInputCloud();

        typename pcl::octree::OctreePointCloud<PointT>::LeafNodeIterator it1;
        typename pcl::octree::OctreePointCloud<PointT>::LeafNodeIterator it1_end = octree1.leaf_end();

        unsigned int leafNodeCounter = 0;


        for (it1 = octree1.leaf_begin(); it1 != it1_end; ++it1) {
            std::vector<int> & indices = it1.getLeafContainer().getPointIndicesVector();

            PointT p;
            Eigen::Map<Eigen::VectorXf> pmap = Eigen::VectorXf::Map(&p.data[0], data_items);

            for(int idx : indices){

                auto & array = *input;
                Eigen::Map<Eigen::VectorXf> pmap1 = Eigen::VectorXf::Map(&(array[idx].data[0]), data_items);
                pmap += pmap1;
                sub_idxs[idx] = output->size();

            }

            float size_inv = 1.0/indices.size();

            pmap*=size_inv;
            output->push_back(p);

            leafNodeCounter++;
        }

        return output;

    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr gridDownsample(std::shared_ptr<PointCloud> input, float resolution, std::vector<int>& sub_idxs);

 private slots:
    void don_vis();
    void hist_vis();
    void fpfh_vis();
    void curve_vis();
    void normalnoise();
    void dist_stdev();
    void sutract_lowfreq_noise();
    void eigen_ratio();
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
