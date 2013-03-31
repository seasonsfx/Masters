#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>

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

class MainWindow : public QMainWindow {
    Q_OBJECT
    
 public:
    explicit MainWindow(QUndoStack * us, CloudList *cl, LayerList *ll, QWidget *parent = 0);
    ~MainWindow();

    ActionManager * getActionManager();
    //const GLWidget getGLWidget();
    //const GLWidget getFlatView();

 public:
    FlatView * flatview_;
    GLWidget * glwidget_;

 private:
    QStatusBar * statusbar_;
    QProgressBar *progressbar_;
    QTabWidget * tabs_;
    CloudListView * clv_;
    LayerListView * llv_;
    GLData * gld_;
    ActionManager * am_;

    QUndoStack * us_;

};

#endif // MAINWINDOW_H
