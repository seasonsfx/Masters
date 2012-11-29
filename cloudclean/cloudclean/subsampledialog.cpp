#include "subsampledialog.h"
#include "ui_subsampledialog.h"

SubsampleDialog::SubsampleDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SubsampleDialog)
{
    ui->setupUi(this);
    subsample = 0;
    connect(ui->spinBox, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
}

SubsampleDialog::~SubsampleDialog()
{
    delete ui;
}

int SubsampleDialog::getSubsample(){
    SubsampleDialog sd;
    if(sd.exec() == QDialog::Accepted){
        if(sd.subsample == 0) sd.subsample = 1;
        return sd.subsample;
    }
    return -1;
}

int SubsampleDialog::valueChanged(int val){
    subsample = val;
    return val;
}

