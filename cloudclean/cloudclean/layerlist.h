#ifndef LAYERLIST_H
#define LAYERLIST_H

#include <QAbstractListModel>
#include <QAbstractItemView>
#include "layer.h"
#include <vector>

class LayerList : public QAbstractListModel
{
    Q_OBJECT
public:

    static const int CREATE_NEW_LAYER = 0;
    static const int ADD_TO_ACTIVE_LAYER = 1;
    static const int REMOVE_POINTS = 2;

    int newLayerMode;


    explicit LayerList(QObject *parent = 0);
    int rowCount ( const QModelIndex & parent = QModelIndex() ) const;
    int columnCount ( const QModelIndex & parent) const;
    Qt::ItemFlags flags ( const QModelIndex & index ) const;
    QVariant data ( const QModelIndex & index, int role = Qt::DisplayRole ) const;
    bool setData(const QModelIndex & index, const QVariant & value, int role);
    void newLayer ();

    void reset();
    std::vector<Layer> layers;
    void activateLayer(int i);
    void toggleVisible(int i);
    void setSelectMode(QAbstractItemView::SelectionMode mode);

private:

signals:
    void selectModeChanged(QAbstractItemView::SelectionMode mode);
    void selectLayer(int i);
    void updateView();

public slots:
    void deleteLayers(std::vector<int> indices);
    void mergeLayers(std::vector<int> layersToMerge);
    void selectModeChanged(int index);

};

#endif // LAYERLIST_H
