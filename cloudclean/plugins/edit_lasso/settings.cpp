#include "settings.h"
#include "ui_settings.h"
#include <cmath>


Settings::Settings(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Settings)
{
    ui->setupUi(this);
    ui->horizontalSlider->setMaximum(1000);
    ui->horizontalSlider->setMinimum(10);
    ui->horizontalSlider->setSliderPosition(1000);

    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(depthChange(int)));

}

Settings::~Settings()
{
    delete ui;
}


float Settings::getDepth(){
    float val = ui->horizontalSlider->value()/(float)ui->horizontalSlider->maximum();
    float steepness = 100;
    float normval = (std::log(val)+steepness)/steepness;
    qDebug("Normval: %f", normval);
    return normval;
}

void Settings::depthChange(int val){
    emit depthChanged(val);
}
