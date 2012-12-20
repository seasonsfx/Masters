#ifndef SUBSAMPLEDIALOG_H
#define SUBSAMPLEDIALOG_H

#include <QDialog>
#include "cloudclean_global.h"

namespace Ui {
class SubsampleDialog;
}

class DLLSPEC SubsampleDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit SubsampleDialog(QWidget *parent = 0);
    ~SubsampleDialog();
    static int getSubsample();

public slots:
    int valueChanged(int val);

private:
    Ui::SubsampleDialog *ui;
    int subsample;
};

#endif // SUBSAMPLEDIALOG_H
