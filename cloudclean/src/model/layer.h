#ifndef MODEL_CLAYER_H_
#define MODEL_CLAYER_H_

#include <set>
#include <QAbstractListModel>
#include <QColor>

//lass CLayer : public QAbstractListModel {
class Layer {
    //Q_OBJECT
 public:
    Layer();

 public:
    QString name_;
    QColor color_;
    //std::set<short> labels_;  // lists cloud lables are associated with this label
    bool is_visible_;  // can this layer be seen on screen
};

#endif // MODEL_CLAYER_H_
