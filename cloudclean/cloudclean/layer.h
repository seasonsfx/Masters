#ifndef LAYER_H
#define LAYER_H

#include <vector>
#include <Eigen/Dense>
#include <QGLBuffer>

class Layer
{
public:
    Layer();
    void toggleActive();
    void toggleVisible();
    void setActive(bool isActive);

    void copyToGPU();
    void copyFromGPU();
    bool sync();

    QGLBuffer gl_index_buffer;
    std::vector<int> index;
    Eigen::Vector3f colour;
    bool visible;
    bool active;

    bool gpu_dirty;
    bool cpu_dirty;
private:
};

#endif // LAYER_H
