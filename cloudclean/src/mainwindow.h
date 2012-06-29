#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QListView>
#include <QMainWindow>
#include <QMdiArea>
#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
#include "glarea.h"
#include "layerview.h"
#include "pluginmanager.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    
signals:
    void reloadCloud();

public slots:
    bool loadScan();
    bool saveScan();

private:
    QMenu *fileMenu;
    QAction *openFile;
    QAction *saveFile;
    LayerView * layers;
    PluginManager pluginManager;

public:
    GLArea * glarea;
};

#endif // MAINWINDOW_H
