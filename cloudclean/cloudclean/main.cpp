#include <QtGui/QApplication>
#include <iostream>
#include "cloudmodel.h"
#include "assert.h"
#include "mainwindow.h"

int main(int argc, char *argv[])
{ 
    QApplication a(argc, argv);

    MainWindow w;
    w.show();
    //w.showMaximized();

    return a.exec();
}
