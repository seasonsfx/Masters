#include "gui/mainwindow.h"

#include <QProgressBar>
#include <QStatusBar>
#include <QAction>
#include <QUndoStack>
#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
#include <QStackedWidget>
#include <QToolBox>
#include <QToolButton>
#include <QToolBar>
#include <QStyle>
#include <QApplication>
#include <QGridLayout>
#include <QBoxLayout>

#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/gldata.h"
#include "gui/cloudlistview.h"
#include "gui/layerlistview.h"
#include "model/cloudlist.h"
#include "model/layerlist.h"

MainWindow::MainWindow(QUndoStack *us, CloudList * cl, LayerList * ll, QWidget *parent)
    : QMainWindow(parent) {

    us_ = us;
    ll_ = ll;
    cl_ = cl;

    progressbar_ = new QProgressBar();
    statusbar_ = statusBar();
    tabs_ = new QTabWidget(this);

    QGLFormat base_format;
    base_format.setVersion(3, 3);
    // Compatibility profile breaks gl context sharing on AMD cards
    //base_format.setProfile(QGLFormat::CompatibilityProfile);
    base_format.setProfile(QGLFormat::CoreProfile);
    base_format.setSampleBuffers(true);

    // Important! Context invalidates when reparenting the glwidget
    glwidget_ = new GLWidget(base_format, cl, ll, tabs_);
    tabs_->addTab(glwidget_, "3D View");
    flatview_ = new FlatView(base_format, cl, ll, tabs_, glwidget_);
    tabs_->addTab(flatview_, "2D View");

    QGLContext * ctx = const_cast<QGLContext *>(glwidget_->context());
    gld_ = new GLData(ctx, cl, ll);
    glwidget_->setGLD(gld_);
    flatview_->setGLD(gld_);

    clv_ = new CloudListView(us, ll, cl, this);
    llv_ = new LayerListView(us, ll, cl, this);

    progressbar_->setTextVisible( false );
    progressbar_->setRange( 0, 100 );

    statusbar_->addWidget( progressbar_, 1 );
    statusbar_->showMessage( tr("Ready"), 2000 );


    QStyle * style = QApplication::style();
    QAction * undo = us_->createUndoAction(0);
    undo->setIcon(style->standardIcon(QStyle::SP_ArrowLeft));
    QAction * redo = us_->createRedoAction(0);
    redo->setIcon(style->standardIcon(QStyle::SP_ArrowRight));

    options_dock_ = new QDockWidget(this);
    options_dock_->setWindowTitle(tr("Tool options"));
    tooloptions_ = new QStackedWidget(options_dock_);
    options_dock_->setWidget(tooloptions_);

    toolbar_ = new QToolBar(this);
    addToolBar(Qt::LeftToolBarArea, toolbar_);
    toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);

    toolbar_->addAction(undo);
    toolbar_->addAction(redo);

    addDockWidget(Qt::RightDockWidgetArea, clv_);
    addDockWidget(Qt::RightDockWidgetArea, llv_);
    tabifyDockWidget(clv_, llv_);
    clv_->raise();

    addDockWidget(Qt::RightDockWidgetArea, options_dock_);
    options_dock_->hide();

    setCentralWidget(tabs_);
    setVisible(true);

    mb_ = menuBar();

    file_menu_ = new QMenu(tr("File"));
    edit_menu_ = new QMenu(tr("Edit"));
    view_menu_ = new QMenu(tr("View"));
    window_menu_ = new QMenu(tr("Window"));

    mb_->addMenu(file_menu_);
    mb_->addMenu(edit_menu_);
    mb_->addMenu(view_menu_);
    mb_->addMenu(window_menu_);

    menus_.insert(tr("File"), file_menu_);
    menus_.insert(tr("Edit"), edit_menu_);
    menus_.insert(tr("View"), view_menu_);
    menus_.insert(tr("Window"), window_menu_);

    QAction * load = new QAction(tr("Load"), this);
    QAction * save = new QAction(tr("Save"), this);
    connect(load, SIGNAL(triggered()), this, SLOT(loadFile()));
    connect(save, SIGNAL(triggered()), this, SLOT(saveFile()));

    file_menu_->addAction(load);
    file_menu_->addAction(save);

    window_menu_->addAction(clv_->toggleViewAction());
    window_menu_->addAction(llv_->toggleViewAction());
    window_menu_->addAction(options_dock_->toggleViewAction());

    // SIGNALS
    qRegisterMetaType<std::shared_ptr<PointCloud> >("std::shared_ptr<PointCloud>");
    qRegisterMetaType<std::shared_ptr<Layer> >("std::shared_ptr<Layer>");

    connect(gld_, SIGNAL(update()), glwidget_, SLOT(update()));
    connect(gld_, SIGNAL(update()), flatview_, SLOT(update()));
    connect(cl, SIGNAL(cloudUpdate(std::shared_ptr<PointCloud>)), gld_, SLOT(reloadCloud(std::shared_ptr<PointCloud>)));
    connect(cl, SIGNAL(updated()), glwidget_, SLOT(update()));
    connect(cl, SIGNAL(updatedActive(std::shared_ptr<PointCloud>)), flatview_, SLOT(setCloud(std::shared_ptr<PointCloud>)));
    connect(cl, SIGNAL(progressUpdate(int)), progressbar_, SLOT(setValue(int)));
    connect(ll, SIGNAL(layerUpdate(std::shared_ptr<Layer>)), gld_, SLOT(reloadColorLookupBuffer()));
    connect(ll, SIGNAL(lookupTableUpdate()), gld_, SLOT(reloadColorLookupBuffer()));

    QAction * deselect = new QAction(tr("Deselect all"), this);
    connect(deselect, SIGNAL(triggered()), clv_, SLOT(deselectAllPoints()));
    edit_menu_->addAction(deselect);

    QAction * select = new QAction(tr("Select all"), this);
    connect(select, SIGNAL(triggered()), clv_, SLOT(selectAllPoints()));
    edit_menu_->addAction(select);

    undo->setShortcut(QKeySequence::Undo);
    redo->setShortcut(QKeySequence::Redo);
    undo->setShortcutContext(Qt::ApplicationShortcut);
    redo->setShortcutContext(Qt::ApplicationShortcut);

    edit_menu_->addAction(undo);
    edit_menu_->addAction(redo);

    gld_->reloadColorLookupBuffer();


    connect(glwidget_, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));

    connect(flatview_, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));

}

MainWindow::~MainWindow() {
    delete gld_;
}

void MainWindow::addMenu(QAction * action, QString menu_name){
    auto it = menus_.find(menu_name);
    QMenu * menu;
    if(it != menus_.end()){
        menu = *it;
    }
    else {
        menu = new QMenu(menu_name, mb_);
        mb_->addMenu(menu);
        menus_.insert(menu_name, menu);
    }
    menu->addAction(action);
}

void MainWindow::removeMenu(QAction * action, QString menu_name){
    auto it = menus_.find(menu_name);
    if(it != menus_.end()){
        QMenu * menu = *it;
        menu->removeAction(action);
    }
}

void MainWindow::loadFile(){
    QString filename = QFileDialog::getOpenFileName(
                 this, tr("Open Scan"), "~", tr("PTX Files (*.ptx)"));
    if (filename.length() == 0)
        return;

    std::thread(&CloudList::loadFile, cl_, filename).detach();
}

void MainWindow::saveFile(){
    QString filename = QFileDialog::getOpenFileName(
                 this, tr("Save Scan"), "~", tr("PTX Files (*.ptx)"));
    if (filename.length() == 0)
        return;

    std::set<uint16_t> slabels;
    for(std::weak_ptr<Layer> wl : ll_->selection_) {
        std::shared_ptr<Layer> l = wl.lock();
        for(uint16_t label : l->getLabelSet()){
            slabels.insert(label);
        }
    }

    std::vector<uint16_t> labels;
    for(uint16_t label : slabels){
        labels.push_back(label);
    }

    std::thread(&CloudList::saveFile, cl_, filename, labels).detach();
}

void MainWindow::contextMenu(const QPoint &pos) {
    QMenu menu;

    menu.addAction("Layer from selection", llv_,
                   SLOT(selectionToLayer()));

    menu.addAction("Deselect all", clv_, SLOT(deselectAllPoints()));
    menu.addAction("Select all", clv_, SLOT(selectAllPoints()));

    menu.exec(glwidget_->mapToGlobal(pos));
}
