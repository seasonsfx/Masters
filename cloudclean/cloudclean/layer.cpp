#include "layer.h"

#include <stdlib.h>
#include <time.h>

inline float rand_range(float from, float to){
    return (rand()/(float)RAND_MAX) * (to-from) + from;
}

Layer::Layer(): gl_index_buffer(QGLBuffer::IndexBuffer)
{
    srand (time(NULL));
    this->active = false;
    this->visible = true;
    this->colour << rand_range(0.2f, 1.0f), rand_range(0.2f, 1.0f), rand_range(0.2f, 1.0f);
}

void Layer::copyToGPU(){
    if(!gl_index_buffer.isCreated())
        return;
    gl_index_buffer.bind();
    gl_index_buffer.write(0, &index[0], gl_index_buffer.size());
}

void Layer::copyFromGPU(){
    if(!gl_index_buffer.isCreated())
        return;

    if(index.size() == 0){
       index.resize(gl_index_buffer.size());
    }
    gl_index_buffer.bind();
    gl_index_buffer.read(0, &index[0], gl_index_buffer.size());
}

void Layer::toggleActive(){
    active = !active;
}

void Layer::toggleVisible(){
    visible = !visible;
}

void Layer::setActive(bool isActive){
    active = isActive;
}
