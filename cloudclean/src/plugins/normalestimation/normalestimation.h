#ifndef NORMAL_ESITIMATION_H
#define NORMAL_ESITIMATION_H

#include "pluginsystem/iplugin.h"

#include <memory>
#include <map>
#include <future>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "plugins/normalestimation/export.h"

#include "glheaders.h"
#ifdef __APPLE__
    #include <OpenCL/opencl.h>
#else
    #include <CL/cl.h>
#endif
#include <CL/cl_gl.h>

class Core;
class CloudList;
class QUndoStack;
class PointCloud;

typedef std::map<std::weak_ptr<PointCloud>, pcl::PointCloud<pcl::Normal>::Ptr,
    std::owner_less<std::weak_ptr<PointCloud>>> NormalMap;

typedef std::map<std::weak_ptr<PointCloud>,
    std::future<pcl::PointCloud<pcl::Normal>::Ptr>,
    std::owner_less<std::weak_ptr<PointCloud>>> FutureNormalMap;

class DLLSPEC NormalEstimator : public IPlugin {
    Q_OBJECT
    Q_INTERFACES(IPlugin)
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    void enablePremptiveEstimation(bool enable);
    pcl::PointCloud<pcl::Normal>::Ptr getNormals(std::shared_ptr<PointCloud> cloud);
    bool normalsAvailible(std::shared_ptr<PointCloud> cloud);

 public slots:
    void addedCloud(std::shared_ptr<PointCloud> cloud);
    void removingCloud(std::shared_ptr<PointCloud> cloud);

 private:
    pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(std::shared_ptr<PointCloud> cloud);

 private:
    bool premptive_estimation_;

    Core * core_;
    CloudList * cl_;
    NormalMap normal_map_;
    FutureNormalMap future_normal_map_;

    cl_context clcontext;
    cl_command_queue cmd_queue;

};

#endif  // NORMAL_ESITIMATION_H
