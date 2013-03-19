#include "model/layer.h"
#include <cstdlib>

Layer::Layer() {
    name_ = "New Layer";
    color_.setHsl(rand()%359, rand()%55 + 200, 127);
}

void Layer::setColor(QColor color){
    color_ = color;
    emit colorChanged();
}

void Layer::setRandomColor(){
    color_.setHsl(rand()%359, rand()%55 + 200, 127);
    emit colorChanged();
}
