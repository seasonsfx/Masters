#ifndef MODEL_CLAYER_H_
#define MODEL_CLAYER_H_

#include <vector>
#include <QAbstractListModel>

//lass CLayer : public QAbstractListModel {
class Layer {
    //Q_OBJECT
 public:
    Layer();

 public:
    std::vector<short> labels;  // lists cloud lables are associated with this label
    bool isVisible;  // can this layer be seen on screen
};

#endif // MODEL_CLAYER_H_
