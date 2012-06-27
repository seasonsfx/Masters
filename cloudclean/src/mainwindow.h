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
    void clickedLayer(const QModelIndex & index);
    void selectLayer(int i);

private:
    QMdiArea * mdiarea;
    QMenu *fileMenu;
    QAction *openFile;
    QAction *saveFile;
    GLArea * glarea;
    LayerView * layers;
};

#endif // MAINWINDOW_H
