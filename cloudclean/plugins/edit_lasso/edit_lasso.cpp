#include "edit_lasso.h"
#include <QIcon>

EditLasso::EditLasso()
{
    editLasso = new QAction(QIcon(":/images/lasso.png"), "Lasso select", this);
    actionList << editLasso;
    foreach(QAction *editAction, actionList)
        editAction->setCheckable(true);
}

EditLasso::~EditLasso()
{
}

void EditLasso::paintGL(){

}

bool EditLasso::mouseDoubleClickEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){
    return true;
}

bool EditLasso::StartEdit(CloudModel * cm, GLArea * glarea){

    if(glarea->activeEditPlugin){
        glarea->activeEditPlugin->EndEdit(cm, glarea);
    }
    glarea->activeEditPlugin = this;

    return true;
}
bool EditLasso::EndEdit(CloudModel *, GLArea *){
    return true;
}

bool EditLasso::mousePressEvent  (QMouseEvent *event, CloudModel *, GLArea * glarea){

}

bool EditLasso::mouseMoveEvent   (QMouseEvent *event, CloudModel *, GLArea * ){

}

bool EditLasso::mouseReleaseEvent(QMouseEvent *event, CloudModel *, GLArea * ){

}

QList<QAction *> EditLasso::actions() const{
    return actionList;
}
QString EditLasso::getEditToolDescription(QAction *){
    return "Info";
}


Q_EXPORT_PLUGIN2(pnp_editlasso, EditLasso)
