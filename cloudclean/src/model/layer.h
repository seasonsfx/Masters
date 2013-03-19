#ifndef MODEL_CLAYER_H_
#define MODEL_CLAYER_H_

#include <set>
#include <QAbstractListModel>
#include <QColor>

class Layer : public QObject {
    Q_OBJECT
 public:
    Layer();
    void setColor(QColor color);
    void setRandomColor();

 signals:
    void colorChanged();

 public:
    QString name_;
    QColor color_;
};

#endif // MODEL_CLAYER_H_
