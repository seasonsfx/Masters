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
class QStackedWidget;
class QToolBox;

class GUI_DLLSPEC MainWindow : public QMainWindow {
    Q_OBJECT
    
 public:
    explicit MainWindow(QUndoStack * us, CloudList *cl, LayerList *ll, QWidget *parent = 0);
    ~MainWindow();

    void addMenu(QAction * action, QString menu_name);
    void removeMenu(QAction * action, QString menu_name);

 public slots:
    void loadFile();
    void saveFile();

    void startBgAction(QString name, bool deterministic = false);
    void stopBgAction(QString name);
    void progBgAction(QString name, int prog);

 private slots:
    void contextMenu(const QPoint &pos);


 public:
   FlatView * flatview_;
   GLWidget * glwidget_;
   QToolBar * toolbar_;
   QStackedWidget * tooloptions_;

   QMenu * file_menu_;
   QMenu * edit_menu_;
   QMenu * view_menu_;
   QMenu * window_menu_;

   QDockWidget * options_dock_;

 protected:
   void closeEvent(QCloseEvent *event);

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
