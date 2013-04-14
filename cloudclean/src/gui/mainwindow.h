#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QHash>
#include <memory>
#include "gui/export.h"

class QUndoStack;
class CloudList;
class LayerList;
class QProgressBar;
class FlatView;
class GLWidget;
class CloudListView;
class LayerListView;
class GLData;
class ActionManager;
class QUndoStack;
class QMenuBar;

class DLLSPEC MainWindow : public QMainWindow {
    Q_OBJECT
    
 public:
    explicit MainWindow(QUndoStack * us, CloudList *cl, LayerList *ll, QWidget *parent = 0);
    ~MainWindow();

    void addMenu(QAction * action, QString menu_name);
    void removeMenu(QAction * action, QString menu_name);

 public:
    FlatView * flatview_;
    GLWidget * glwidget_;

 public slots:
    void loadFile();
    void saveFile();

 private:
    QStatusBar * statusbar_;
    QProgressBar *progressbar_;
    QTabWidget * tabs_;
    CloudListView * clv_;
    LayerListView * llv_;
    GLData * gld_;
    CloudList * cl_;
    LayerList * ll_;

    QUndoStack * us_;

    QMenuBar * mb_;
    QHash<QString, QMenu *> menus_;

};

#endif // MAINWINDOW_H
