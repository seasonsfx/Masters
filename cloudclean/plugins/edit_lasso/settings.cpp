#include "settings.h"
#include "ui_settings.h"

Settings::Settings(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Settings)
{
    ui->setupUi(this);
    ui->horizontalSlider->setMaximum(1000);
    ui->horizontalSlider->setMinimum(10);
    ui->horizontalSlider->setSliderPosition(1000);
}

Settings::~Settings()
{
    delete ui;
}


float Settings::getDepth(){
    float max = ui->horizontalSlider->maximum();
    return ui->horizontalSlider->value()/max;
}
