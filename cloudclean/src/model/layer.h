#ifndef MODEL_CLAYER_H_
#define MODEL_CLAYER_H_

#include <set>
#include <map>
#include <memory>
#include <QAbstractListModel>
#include <QColor>
#include "model/export.h"

class Layer;
class LayerList;
typedef std::set<Layer *> LayerSet;

class MODEL_DLLSPEC Layer : public QObject {
    Q_OBJECT
 private:
    Layer(std::map<uint16_t, LayerSet> & layer_lookup_table);

 public:
    ~Layer();
    void setColor(QColor color);
    void setRandomColor();
    void setName(QString name);
    void addLabel(uint16_t id);
    void removeLabel(uint16_t id);
    const std::set<uint16_t> &getLabelSet();

    bool isVisible() const;
    void toggleVisible();

 signals:
    void colorChanged();
    void nameChanged();

 private:
    std::map<uint16_t, LayerSet> & layer_lookup_table_;
    std::set<uint16_t> labels_;

    static uint last_id_;

    bool visible_;

 public:
    QString name_;
    QColor color_;
    uint id_;

    friend class LayerListView;
    friend class LayerList;
};

#endif // MODEL_CLAYER_H_
