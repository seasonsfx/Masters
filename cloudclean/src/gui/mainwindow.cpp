#include "gui/mainwindow.h"

#include <QProgressBar>
#include <QStatusBar>
#include <QAction>
#include <QUndoStack>
#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>

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

    qDebug() << "widget:" << glwidget_->context();
    qDebug() << "flatview:" << flatview_->context();
    qDebug() << "is sharing" << QGLContext::areSharing(flatview_->context(), glwidget_->context());

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


    addDockWidget(Qt::RightDockWidgetArea, clv_);
    addDockWidget(Qt::RightDockWidgetArea, llv_);

    setCentralWidget(tabs_);
    setVisible(true);

    mb_ = menuBar();

    QAction * load = new QAction(tr("Load"), this);
    QAction * save = new QAction(tr("Save"), this);
    connect(load, SIGNAL(triggered()), this, SLOT(loadFile()));
    connect(save, SIGNAL(triggered()), this, SLOT(saveFile()));
    addMenu(load, "File");
    addMenu(save, "File");

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
    addMenu(deselect, "Edit");


    QAction * undo = us_->createUndoAction(0);
    QAction * redo = us_->createRedoAction(0);
    undo->setShortcut(QKeySequence::Undo);
    redo->setShortcut(QKeySequence::Redo);
    undo->setShortcutContext(Qt::ApplicationShortcut);
    redo->setShortcutContext(Qt::ApplicationShortcut);
    addMenu(undo, "Edit");
    addMenu(redo, "Edit");


    gld_->reloadColorLookupBuffer();
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
