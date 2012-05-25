#include <QtGui/QApplication>
#include "glwidget.h"
#include "cloudclean.h"
#include <iostream>
#include "appdata.h"
#include "assert.h"

int main(int argc, char *argv[])
{ 
    //assert(argc == 3);

    AppData * app_data = AppData::Instance();

    if(argc == 3){
        int subsample = atoi(argv[1]);
        char* input_file = argv[2];
        app_data->loadFile(input_file, subsample);
    }
    QApplication a(argc, argv);

    CloudClean w;
    w.show();

    return a.exec();
}
