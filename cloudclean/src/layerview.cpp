#include "layerview.h"
#include "cloudmodel.h"

LayerView::LayerView(QWidget * parent, Qt::WindowFlags flags)
{
    QDockWidget(parent, flags);
    layers = new QListView(this);
    layers->setModel(&CloudModel::Instance()->layerList);
    layers->setSelectionMode(QAbstractItemView::MultiSelection);
}
