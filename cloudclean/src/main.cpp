#include "app.h"
#include <ctime>
#include <cstdlib>
#include <QCoreApplication>
#include <QtDebug>
#include <QtGlobal>

#ifdef WIN32
    #include <Windows.h>
#endif

void customMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString &msg)
{
    QString txt;

    txt = QString("MESSAGE (%1:%2 %3):\n\t %4\n").arg(context.file).arg(context.line).arg(context.function).arg(msg);

    QFile outFile("debuglog.txt");
    outFile.open(QIODevice::WriteOnly | QIODevice::Append);
    QTextStream ts(&outFile);
    ts << txt << endl;

#ifdef WIN32
    OutputDebugString(txt.toLocal8Bit().constData());
#else
    //printf("MESSAGE (%s:%u %s): %s\n", context.file, context.line, context.function, msg.constData());
    printf("%s", txt.toLocal8Bit().constData());
    fflush(stdout);
#endif


}

int main(int argc, char* argv[])
{
    srand (time(NULL));
    qInstallMessageHandler(customMessageHandler);
    App app(argc,argv);
    int status = app.exec();
    qInstallMessageHandler(0);
    return status;
}

