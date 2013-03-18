#ifndef MODEL_CPOINTCLOUD_H_
#define MODEL_CPOINTCLOUD_H_

#include <mutex>
#include <memory>
#include <QObject>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

class PointCloud;

class EventDispatcher : public QObject {
    Q_OBJECT
 public:
    EventDispatcher(PointCloud * pc);

 private:
    void updateProgress(int value);
    void emitlabelUpdate(std::shared_ptr<std::vector<int> > idxs = nullptr);
    void emitTransformed(std::shared_ptr<std::vector<int> > idxs = nullptr);

 public:
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

enum class PointFlags : int8_t {
    selected = 0x001,
    reserved1 = 0x002,
    reserved2 = 0x006,
    reserved3 = 0x008,
    reserved4 = 0x010,
    reserved5 = 0x020,
    reserved6 = 0x040,
};

enum class CoordinateFrame: bool {
    Camera,
    Laser,
};

class PointCloud : public pcl::PointCloud<pcl::PointXYZI> {
 public:
    explicit PointCloud();
    bool save_ptx(const char* filename);
    bool load_ptx(const char* filename, int decimation_factor = 1);

    void translate(const Eigen::Vector3f& pos);
    void translate(float x, float y, float z)  {
        translate(Eigen::Vector3f(x, y, z));
    }
    void rotate2D(float x, float y);

    Eigen::Affine3f modelview();

 public:
    std::shared_ptr<EventDispatcher> ed_;
    std::shared_ptr<std::mutex> pc_mutex;
    std::vector<int> cloud_to_grid_map_;
    int scan_width_;
    int scan_height_;
    std::vector<int16_t> labels_;
    std::vector<PointFlags> flags_;

    CoordinateFrame frame_;
    // What needs to be in here?
    // Should normals be a default requirement
    // Maybe store attributes in a map and generate on the fly
};

#endif // MODEL_CPOINTCLOUD_H_
