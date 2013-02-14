#ifndef MODEL_CLAYER_H_
#define MODEL_CLAYER_H_

#include <set>
#include <QAbstractListModel>
#include <QColor>

class Layer {
    //Q_OBJECT
 public:
    Layer();

 public:
    QString name_;
    QColor color_;
};

#endif // MODEL_CLAYER_H_
