#ifndef LAYER_H
#define LAYER_H

#include <vector>
#include <Eigen/Dense>
#include <QGLBuffer>

class Layer
{
public:
    Layer();
    //void updateCPU();
    //void updateGPU();
private:
    QGLBuffer gl_index_buffer;
    std::vector<int> index;
    Eigen::Vector3f colour;
    bool visible;
};

#endif // LAYER_H
