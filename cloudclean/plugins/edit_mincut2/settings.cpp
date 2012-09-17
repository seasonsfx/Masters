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

int Settings::kConnectvity(){
    return ui->kNeigbours->value();
}

float Settings::sigma(){
    return ui->sigma->value();
}

float Settings::radius(){
    return ui->radius->value();
}

bool Settings::horisontalRadius(){
    return ui->isHorisonal->isChecked();
}

float Settings::sourceWeight(){
    return ui->sourceWeight->value();
}
