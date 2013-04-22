#ifndef MODEL_CPOINTCLOUD_H_
#define MODEL_CPOINTCLOUD_H_

#include <mutex>
#include <memory>
#include <future>
#include <thread>
#include <QObject>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/octree.h>
#include <Eigen/Dense>
#include "model/export.h"


class PointCloud;

typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> Octree;

class MODEL_DLLSPEC EventDispatcher : public QObject {
    Q_OBJECT
 public:
    EventDispatcher(PointCloud * pc);

 private:
    void updateProgress(int value);
    void emitTransformed(std::shared_ptr<std::vector<int> > idxs = nullptr);

 public:
    void emitlabelUpdate(std::shared_ptr<std::vector<int> > idxs = nullptr);
    void emitflagUpdate(std::shared_ptr<std::vector<int> > idxs = nullptr);

 signals:
    void transformed();
    void progress(int percentage);
    void flagUpdate(std::shared_ptr<std::vector<int> > idxs = nullptr);
    void labelUpdate(std::shared_ptr<std::vector<int> > idxs = nullptr);

 public slots:
   void resetOrientation();

 private:
   PointCloud * pc_;

 friend class PointCloud;
};

enum class MODEL_DLLSPEC PointFlags : int8_t {
    selected = 0x001,
    reserved1 = 0x002,
    reserved2 = 0x006,
    reserved3 = 0x008,
    reserved4 = 0x010,
    reserved5 = 0x020,
    reserved6 = 0x040
};

enum class MODEL_DLLSPEC CoordinateFrame: bool {
    Camera,
    Laser
};

class MODEL_DLLSPEC PointCloud : public pcl::PointCloud<pcl::PointXYZI> {
 public:
    explicit PointCloud();
    ~PointCloud();
    bool point_matches_label(int idx, std::vector<uint16_t> & labels);
    bool save_ptx(const char* filename, std::vector<uint16_t> labels);
    bool load_ptx(const char* filename, int decimation_factor = 1);

    void translate(const Eigen::Vector3f& pos);
    void translate(float x, float y, float z)  {
        translate(Eigen::Vector3f(x, y, z));
    }
    void rotate2D(float x, float y);

    Eigen::Affine3f modelview();
    const Octree::Ptr getOctree();

 private:
    std::future<Octree::Ptr> fut_octree;
    Octree::Ptr octree;

 public:
    std::shared_ptr<EventDispatcher> ed_;
    std::shared_ptr<std::mutex> pc_mutex;
    std::vector<int> cloud_to_grid_map_;
    int scan_width_;
    int scan_height_;
    std::vector<int16_t> labels_;
    std::vector<PointFlags> flags_;

    CoordinateFrame frame_;

    // yeah... i think this is in the octree
    Eigen::Vector3f min_bounding_box_;
    Eigen::Vector3f max_bounding_box_;

    bool deleting_;

};

#endif // MODEL_CPOINTCLOUD_H_
