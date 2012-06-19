#include "layer.h"

#include <stdlib.h>
#include <time.h>

inline float rand_range(float from, float to){
    return (rand()/(float)RAND_MAX) * (to-from) + from;
}

Layer::Layer(): gl_index_buffer(QGLBuffer::IndexBuffer)
{
    srand (time(NULL));
    this->visible = true;
    this->colour << rand_range(0.2f, 1.0f), rand_range(0.2f, 1.0f), rand_range(0.2f, 1.0f);
}
