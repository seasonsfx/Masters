#ifndef EDITLASSO_H
#define EDITLASSO_H

#include <QtPlugin>
#include <QObject>
#include "interfaces.h"

class EditLasso : public QObject, public EditPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(EditPluginInterface)
public:
    EditLasso();
    //bool StartEdit(CloudModel &, GLArea *){return true;}
    //bool EndEdit(CloudModel &, GLArea *){return true;}
    void mousePressEvent  (QMouseEvent *event, CloudModel &, GLArea * );
    void mouseMoveEvent   (QMouseEvent *event, CloudModel &, GLArea * );
    void mouseReleaseEvent(QMouseEvent *event, CloudModel &, GLArea * );
    //void keyReleaseEvent  (QKeyEvent *, CloudModel &, GLArea *){}
    //void keyPressEvent    (QKeyEvent *, CloudModel &, GLArea *){}
    //void wheelEvent(QWheelEvent*, CloudModel &, GLArea * ){}
};

#endif // EDITLASSO_H
