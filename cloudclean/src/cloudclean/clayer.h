#ifndef CLAYER_H
#define CLAYER_H

#include <vector>
#include <QAbstractListModel>

//class CLayer : public QAbstractListModel {
class CLayer {
    //Q_OBJECT
 public:
    CLayer();

 public:
    std::vector<short> labels;  // lists cloud lables are associated with this label
    bool isVisible;  // can this layer be seen on screen
};

#endif // CLAYER_H
