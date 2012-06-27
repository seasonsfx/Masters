#include <QtGui/QApplication>
//#include "cloudclean.h"
#include <iostream>
#include "cloudmodel.h"
#include "assert.h"
#include "mainwindow.h"

int main(int argc, char *argv[])
{ 
    QApplication a(argc, argv);

    //CloudModel * app_data = CloudModel::Instance();
    /*if(argc == 3){
        int subsample = atoi(argv[1]);
        char* input_file = argv[2];
        app_data->loadFile(input_file, subsample);
    }
    */

    MainWindow w;
    w.show();
    w.showMaximized();

    return a.exec();
}
