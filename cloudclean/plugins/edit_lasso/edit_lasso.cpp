#include "edit_lasso.h"

EditLasso::EditLasso()
{
}

EditLasso::~EditLasso()
{
}

void EditLasso::mousePressEvent  (QMouseEvent *event, CloudModel &, GLArea * ){

}

void EditLasso::mouseMoveEvent   (QMouseEvent *event, CloudModel &, GLArea * ){

}

void EditLasso::mouseReleaseEvent(QMouseEvent *event, CloudModel &, GLArea * ){

}

Q_EXPORT_PLUGIN2(pnp_editlasso, EditLasso)
