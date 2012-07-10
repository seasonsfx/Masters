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
#include "toolbox.h"
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
    void applyEditMode();

private:
    QMenu *fileMenu;
    QMenu *toolsMenu;
    QAction *openFile;
    QAction *saveFile;
    LayerView * layerView;
    Toolbox * toolbox;
    PluginManager pluginManager;

public:
    GLArea * glarea;
};

#endif // MAINWINDOW_H
