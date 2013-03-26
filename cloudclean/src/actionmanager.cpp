#include "actionmanager.h"
#include <QMenu>
#include <QMenuBar>
#include <QAction>

ActionManager::ActionManager(QMenuBar * mb){
    mb_ = mb;
}

void ActionManager::addAction(QAction * action, QString menu_name){
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

void ActionManager::removeAction(QAction * action, QString menu_name){
    auto it = menus_.find(menu_name);
    if(it != menus_.end()){
        QMenu * menu = *it;
        menu->removeAction(action);
    }
}
