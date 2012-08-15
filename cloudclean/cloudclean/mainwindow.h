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
    bool loadScan(char * filename, int subsample);
    
signals:
    void reloadCloud();
    void setSettingsWidget(QWidget *);
public slots:
    bool loadScan();
    bool saveScan();
    void applyEditMode();
    void applyVizMode();

private:
    QMenu *fileMenu;
    QMenu *toolsMenu;
    QMenu *vizMenu;
    QAction *openFile;
    QAction *saveFile;
    LayerView * layerView;
    Toolbox * toolbox;
    CloudModel * cm;
    PluginManager pm;

public:
    GLArea * glarea;
};

#endif // MAINWINDOW_H
