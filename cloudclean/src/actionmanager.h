#ifndef ACTIONMANAGER_H
#define ACTIONMANAGER_H

#include <QObject>
#include <QHash>
#include <QString>

class QMenuBar;
class QAction;
class QMenu;

class ActionManager: public QObject {
    Q_OBJECT
 public:
    ActionManager(QMenuBar * mb);
    void addAction(QAction * action, QString menu_name);
    void removeAction(QAction * action, QString menu_name);
 private:
    QMenuBar * mb_;
    QHash<QString, QMenu *> menus_;
};

#endif  // ACTIONMANAGER_H
