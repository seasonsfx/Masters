#ifndef LAYER_H
#define LAYER_H

#include <vector>
#include <Eigen/Dense>
#include <QGLBuffer>

class Layer
{
public:
    Layer();
    void copyToGPU();
    void copyFromGPU();
    void toggleActive();
    void toggleVisible();
    void setActive(bool isActive);

    QGLBuffer gl_index_buffer;
    std::vector<int> index;
    Eigen::Vector3f colour;
    bool visible;
    bool active;
private:
};

#endif // LAYER_H
