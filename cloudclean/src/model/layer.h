#ifndef MODEL_CLAYER_H_
#define MODEL_CLAYER_H_

#include <set>
#include <memory>
#include <QAbstractListModel>
#include <QColor>


class Layer;
class LayerList;
typedef std::set<Layer *> LayerSet;

class Layer : public QObject {
    Q_OBJECT
 private:
    Layer(std::map<uint8_t, LayerSet> & layer_lookup_table);

 public:
    ~Layer();
    void setColor(QColor color);
    void setRandomColor();
    void setName(QString name);
    void addLabel(uint8_t id);
    void removeLabel(uint8_t id);
    const std::set<uint8_t> &getLabelSet();

 signals:
    void colorChanged();
    void nameChanged();

 private:
    std::map<uint8_t, LayerSet> & layer_lookup_table_;
    std::set<uint8_t> labels_;

 public:
    QString name_;
    QColor color_;


    friend class LayerList;
};

#endif // MODEL_CLAYER_H_
