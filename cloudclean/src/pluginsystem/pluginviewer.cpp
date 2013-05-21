#include "pluginsystem/pluginviewer.h"
#include <QListWidget>

PluginViewer::PluginViewer(QWidget *parent)
    : QWidget(parent) {
    QListWidget *listWidget = new QListWidget(this);
    QListWidgetItem *newItem = new QListWidgetItem;
    newItem->setText("itemText");
    listWidget->insertItem(0, newItem);
}
