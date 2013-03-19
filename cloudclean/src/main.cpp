#include "app.h"
#include <ctime>
#include <cstdlib>

int main(int argc, char* argv[])
{
    srand (time(NULL));
    App app(argc,argv);
    return app.exec();
}

