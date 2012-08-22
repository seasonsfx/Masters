#include "toolbox.h"
#include "ui_toolbox.h"

Toolbox::Toolbox(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::Toolbox)
{
    ui->setupUi(this);
}

Toolbox::~Toolbox()
{
    delete ui;
}

void Toolbox::setSettingsWidget(QWidget * widget){
    //ui->settingsArea->setWidget(widget);
    ui->content->addWidget(widget);
    this->show();
}
