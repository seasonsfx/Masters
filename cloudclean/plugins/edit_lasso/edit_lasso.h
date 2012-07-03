#ifndef EDITLASSO_H
#define EDITLASSO_H

#include <QObject>
#include <QAction>
#include "../../common/interfaces.h"

class GLArea;

class EditLasso : public QObject, public EditPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(EditPluginInterface)
public:
    EditLasso();
    ~EditLasso();
    bool StartEdit(CloudModel * cm, GLArea * glarea);
    bool EndEdit(CloudModel *, GLArea * glarea);
    virtual void paintGL();
    bool mouseDoubleClickEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea);
    bool mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea);
    bool mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * );
    bool mouseReleaseEvent(QMouseEvent *event, CloudModel *, GLArea * );
    //bool keyReleaseEvent  (QKeyEvent *, CloudModel *, GLArea *){}
    //bool keyPressEvent    (QKeyEvent *, CloudModel *, GLArea *){}
    //bool wheelEvent(QWheelEvent*, CloudModel *, GLArea * ){}
    QList<QAction *> actions() const;
    QString getEditToolDescription(QAction *);

private:
    QList <QAction *> actionList;
    QAction *editLasso;

};

#endif // EDITLASSO_H
