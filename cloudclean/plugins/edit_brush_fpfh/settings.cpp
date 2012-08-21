#include "settings.h"
#include "ui_settings.h"

Settings::Settings(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Settings)
{
    ui->setupUi(this);

    distanceFunc = EUCLIDIAN;
    neighbourFunc = K;

    ui->cosine->hide();
    ui->intensity->hide();

    ui->fixed->hide();
    ui->dyn->hide();


    connect(ui->threshBox, SIGNAL(toggled(bool)), this, SLOT(changeDistFunc()));
    connect(ui->neighbourBox, SIGNAL(toggled(bool)), this, SLOT(changeNeigbourFunc()));

    connect(ui->cosineRad, SIGNAL(toggled(bool)), this, SLOT(changeDistFunc()));
    connect(ui->intensityRad, SIGNAL(toggled(bool)), this, SLOT(changeDistFunc()));
    connect(ui->euclidianRad, SIGNAL(toggled(bool)), this, SLOT(changeDistFunc()));

    connect(ui->kRad, SIGNAL(toggled(bool)), this, SLOT(changeNeigbourFunc()));
    connect(ui->fixedRad, SIGNAL(toggled(bool)), this, SLOT(changeNeigbourFunc()));
    connect(ui->dynRad, SIGNAL(toggled(bool)), this, SLOT(changeNeigbourFunc()));

}

Settings::~Settings()
{
    delete ui;
}

void Settings::changeDistFunc(){
    if(ui->euclidianRad->isChecked()){
        distanceFunc = EUCLIDIAN;
        ui->cosine->hide();
        ui->intensity->hide();
        ui->euclidian->show();
    }
    else if(ui->cosineRad->isChecked()){
        distanceFunc = COSINE;
        ui->euclidian->hide();
        ui->intensity->hide();
        ui->cosine->show();
    }
    else if(ui->intensityRad->isChecked()){
        distanceFunc = INTENSITY;
        ui->cosine->hide();
        ui->euclidian->hide();
        ui->intensity->show();
    }
}

void Settings::changeNeigbourFunc(){
    if(ui->kRad->isChecked()){
        neighbourFunc = K;
        ui->fixed->hide();
        ui->dyn->hide();
        ui->k->show();
    }
    else if(ui->fixedRad->isChecked()){
        neighbourFunc = FIXEDRAD;
        ui->k->hide();
        ui->dyn->hide();
        ui->fixed->show();
    }
    else if(ui->dynRad->isChecked()){
        neighbourFunc = DYNRAD;
        ui->fixed->hide();
        ui->k->hide();
        ui->dyn->show();
    }
}

int Settings::kNeigbours() const{ return ui->k->value();}
float Settings::fixedRad() const{ return ui->fixed->value();}
float Settings::dynRad() const{ return ui->dyn->value();}

int Settings::euclidianDist() const{ return ui->euclidian->value();}
float Settings::cosineDist() const{ return ui->cosine->value();}
float Settings::intensityDist() const{ return ui->intensity->value();}
