#include "core.h"
#include <QAction>
#include <QUndoStack>

#include "gui/mainwindow.h"
#include <model/cloudlist.h>
#include <model/layerlist.h>

Core::Core() {
    us_ = new QUndoStack();
    ll_ = new LayerList();
    cl_ = new CloudList();
    mw_ = new MainWindow(us_, cl_, ll_);
}

Core::~Core() {
    delete us_;
    delete ll_;
    delete cl_;
    delete mw_;
}
