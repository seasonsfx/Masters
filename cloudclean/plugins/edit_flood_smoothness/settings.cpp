#include "settings.h"
#include "ui_settings.h"

Settings::Settings(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Settings)
{
    ui->setupUi(this);
}

Settings::~Settings()
{
    delete ui;
}

float Settings::maxNoise(){
    return ui->maxNoise->value();
}

float Settings::minNoise(){
    return ui->minNoise->value();
}

int Settings::kConnectvity(){
    return ui->kConnect->value();
}

int Settings::kNoise(){
    return ui->noiseK->value();
}

